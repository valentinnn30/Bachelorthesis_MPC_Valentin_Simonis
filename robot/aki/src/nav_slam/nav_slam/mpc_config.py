
# this is the 4DoF model (directions and yaw), above the same but with 6 DoF.
# Pumps only for horizontal, bcu only for vertical


from casadi import *
import do_mpc
import numpy as np
import os
import sys

# If needed (for local usage of do-mpc):
#rel_do_mpc_path = os.path.join('..','..','..')
#sys.path.append(rel_do_mpc_path)


def get_model():
    model_type = 'continuous'
    m = do_mpc.model.Model(model_type)

    # === STATES ===
    eta = m.set_variable(var_type='_x', var_name='eta', shape=(4,1))  # [x, y, z, yaw]
    nu = m.set_variable(var_type='_x', var_name='nu', shape=(4,1))    # [u, v, w, r] (linear velocities and yaw rate)

    BCU_position = m.set_variable(var_type='_x', var_name='BCU_position', shape=(1,1))  # Position of the BCU



    # === INPUTS ===
    T = m.set_variable(var_type='_u', var_name='T', shape=(4,1))  # thrusts from MPC
    BCU_INPUT = m.set_variable(var_type='_u', var_name='BCU_INPUT', shape=(1,1))  # Buoyancy control input [50, 100]

    # === Disturbances ===

    d_ext = m.set_variable(var_type='parameter', var_name='d_ext', shape=(4,1))  # External disturbances: [dx, dy, dz, dr]

    # === PARAMETERS ===
    # Rigid body mass and inertia
    mass = m.set_variable('parameter', 'mass')
    Izz = m.set_variable('parameter', 'Izz')  # Only Izz is needed for yaw

    # === PARAMETERS TO ESTIMATE === (added mass and damping)
    X_udot = m.set_variable('parameter', 'X_udot')
    Y_vdot = m.set_variable('parameter', 'Y_vdot')
    Z_wdot = m.set_variable('parameter', 'Z_wdot')
    N_rdot = m.set_variable('parameter', 'N_rdot')

    # Damping (linear and quadratic)
    d_lin = [m.set_variable('parameter', f'd_lin_{i}') for i in range(4)]  # Linear damping
    d_quad = [m.set_variable('parameter', f'd_quad_{i}') for i in range(4)]  # Quadratic damping


    k_rate = m.set_variable('parameter', 'k_rate')  # Constant rate of change for the BCU


    # === MASS MATRIX ===
    M = SX.zeros(4,4)
    M[0,0] = mass - X_udot
    M[1,1] = mass - Y_vdot
    M[2,2] = mass - Z_wdot
    M[3,3] = Izz - N_rdot

    # === Coriolis & Centripetal MATRIX ===
    C = SX.zeros(4,4)
    C[0,3] = (-mass + Y_vdot)*nu[1]
    C[1,3] = (mass - X_udot)*nu[0]
    C[3,0] = (mass - Y_vdot)*nu[1]
    C[3,1] = (-mass + X_udot)*nu[0]

    C_nu = C @ nu

    # === ROTATION MATRICES ===
    psi = eta[3]  # yaw remains variable

    # Simplified rotation matrix for reduced state (only yaw)
    R = vertcat(
        horzcat(cos(psi), -sin(psi), 0),
        horzcat(sin(psi), cos(psi), 0),
        horzcat(0, 0, 1)
    )

    # Angular Jacobian J (simplified for only yaw)
    J = vertcat(
        horzcat(1, 0),
        horzcat(0, 1)
    )

    # Kinematics
    eta_dot = vertcat(R @ nu[0:3], nu[3])  # Only yaw angular velocity remains

    # === DAMPING ===
    D_lin = diag(vertcat(*d_lin))
    D_quad = diag(vertcat(*d_quad))

    # Nonlinear damping
    D_nu = D_lin @ nu + D_quad @ (nu * fabs(nu))

    # === THRUSTER DIRECTIONS ===
    theta_rad = np.deg2rad(20) # Angle of T1, T2, T3 with the horizontal plane
    azimuths = [0, 2*np.pi/3, 4*np.pi/3]
    dirs = []

    for a in azimuths:
        dir_x = np.cos(a) * np.cos(theta_rad)
        dir_y = np.sin(a) * np.cos(theta_rad)
        dir_z = np.sin(theta_rad)  # upwards
        dirs.append([dir_x, dir_y, dir_z])

    # Direction for T4: purely in the negative Z direction
    dirs.append([0, 0, -1]) # [Fx, Fy, Fz] for T4 - 0 in X, 0 in Y, -1 in Z

    D_mat = DM(dirs).T  # shape (3,3)

    # Compute total body-frame force from thrusters
    f_xyz = D_mat @ T  # [Fx, Fy, Fz]

    m.set_expression(expr_name='f_xyz', expr=f_xyz)

    # Buoyancy control force Fz_x based on a (adjusted linearly between -1.6N and 1.6N)
    BCU_dot = k_rate * (BCU_INPUT - BCU_position)  # Change in position based on setpoint a and current position
    Fz_x = (BCU_position - 75) * 0.0984 # with 1.6N a value of 0.064
    m.set_expression(expr_name='Fz_x_output', expr=Fz_x)
    
    # Thruster forces and torques
    tau = vertcat(
        f_xyz[0],
        f_xyz[1],
        f_xyz[2] + Fz_x,
        SX(0)  # No torque for roll and pitch since they're fixed
    )

    # === DYNAMICS ===
    # nu_dot = inv(M) @ (tau - D_nu)
    nu_dot = inv(M) @ (tau - D_nu - C_nu + d_ext)

    # === RIGHT-HAND SIDES ===
    m.set_rhs('eta', eta_dot)
    m.set_rhs('nu', nu_dot)
    m.set_rhs('BCU_position', BCU_dot)
    # m.set_rhs('d_ext', SX.zeros(4,1))  # Constant disturbance over the prediction horizon


    m.setup()

    return m



import casadi as ca
def get_mpc(model, reference_goal):
    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 20,
        't_step': 0.2,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 2,
        'store_full_solution': True,
        'n_robust': 1, # use some other values to have uncertainty parameters inside the model
    }

    mpc.set_param(**setup_mpc)


    # Define reference values for eta and nu
    #ref_eta = ca.DM([5, 4, -3, 0]).reshape((4, 1))
    #ref_nu  = ca.DM([0, 0, 0, 0]).reshape((4, 1))

    ref_eta = ca.DM([reference_goal['x'], reference_goal['y'], reference_goal['depth'], reference_goal['yaw']]).reshape((4, 1))
    ref_nu  = ca.DM([0.0, 0.0, 0.0, 0.0]).reshape((4, 1))


    # Define terminal and stage costs
    mterm = (model.x['eta'] - ref_eta)**2 + (model.x['nu'] - ref_nu)**2

    # --- Penalties ---
    Z_FORCE_PENALTY_WEIGHT = 50.0 # High penalty for Z-force from thrusters (adjust as needed)
    THRUSTER_USE_PENALTY_WEIGHT = 0.01 # General penalty for thruster usage
    W_NU_Z = 5 # penalizes speed of robot, should move down slow to avoid overshoot due to high inertia
    #lterm = ca.sum1(mterm) + THRUSTER_USE_PENALTY_WEIGHT * ca.sum1(model.u['T']**2) + 0.01 * ca.sum1(model.u['BCU_INPUT']**2) + Z_FORCE_PENALTY_WEIGHT * (model.aux['f_xyz'][2]**2)
    # not penalize BCU, makes no sense, outmax is nice. Extended BCU is same as retracted -> maybe nice to be closer to neutrally buoyant, but then transform to forces!
    lterm = ca.sum1(mterm) + THRUSTER_USE_PENALTY_WEIGHT * ca.sum1(model.u['T']**2) + Z_FORCE_PENALTY_WEIGHT * (model.aux['f_xyz'][2]**2) + W_NU_Z * (model.x['nu'][2]**2)

    # Sum to scalar
    mpc.set_objective(mterm=ca.sum1(mterm), lterm=lterm)

    # Input constraints
    # for pumps directly the Thrust, canbe switched later to match ros data
    mpc.bounds['lower','_u','T'] = 0.0      # Minimum Thrust per Thruster
    mpc.bounds['upper','_u','T'] = 5.58     # Maximum Thrust per Thruster -> calculating by finding the root
    # for the BCUs we constraint between 50 and 100, not the real forces, since a linear mapping exists
    mpc.bounds['lower','_u','BCU_INPUT'] = 50   # BCU force to move down
    mpc.bounds['upper','_u','BCU_INPUT'] = 100    # BCU force to move down
    #mpc.bounds['lower', '_x', 'eta'][4] = -np.deg2rad(80)
    #mpc.bounds['upper', '_x', 'eta'][4] =  np.deg2rad(80)

    # Model disturbances
    d_ext_min = ca.DM([-2.0, -2.0, -0.5, -0.3]).reshape((4,1))
    d_ext_max = ca.DM([2.0,  2.0,  0.5, 0.3]).reshape((4,1))
    d_ext_nominal = ca.DM([0, 0, 0, 0]).reshape((4,1))

    d_ext_uncertainty_scenarios = [
      d_ext_min,
      d_ext_nominal,
      d_ext_max
    ]





    mpc.set_uncertainty_values(
        k_rate = [0.25],
        mass = [19.16],
        Izz = [0.285],
        X_udot = [-18.0],
        Y_vdot = [-18.0],
        Z_wdot = [-25.0],
        N_rdot = [-0.5],
        d_lin_0 = [2.0],
        d_lin_1 = [2.0],
        d_lin_2 = [3.0],
        d_lin_3 = [0.1],
        d_quad_0 = [13.0],
        d_quad_1 = [13.0],
        d_quad_2 = [28.0],
        d_quad_3 = [0.1],
        d_ext = d_ext_uncertainty_scenarios
        #W = [144.2],
        #B = [144.2],
        #z_g = [0.0],
        #z_b = [0.2]
    )



    mpc.setup()
    return mpc


def get_mpc_bcuonly(model, reference_goal):
    mpc = do_mpc.MPC(model)

    setup_mpc = {
        'n_horizon': 20,
        't_step': 0.2,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 2,
        'store_full_solution': True,
        'n_robust': 1, # use some other values to have uncertainty parameters inside the model
    }

    mpc.set_param(**setup_mpc)

    ref_eta = ca.DM([reference_goal['x'], reference_goal['y'], reference_goal['depth'], reference_goal['yaw']]).reshape((4, 1))
    ref_nu  = ca.DM([0.0, 0.0, 0.0, 0.0]).reshape((4, 1))

    # --- mterm (Terminal Cost) ---
    # Focus heavily on final depth (z) and zero vertical velocity (w)
    mterm = (model.x['eta'][2] - ref_eta[2])**2 * 1000000  # Very high weight for depth (z)
    mterm += model.x['nu'][2]**2 * 10000 # Very high weight for zero vertical velocity (w) at terminal state

    # Keep very small or zero weights for X, Y, Yaw, and their body velocities (u, v, r)
    mterm += (model.x['eta'][0] - ref_eta[0])**2 * 0.001  # x
    mterm += (model.x['eta'][1] - ref_eta[1])**2 * 0.001  # y
    mterm += (model.x['eta'][3] - ref_eta[3])**2 * 0.001  # yaw
    mterm += model.x['nu'][0]**2 * 0.001  # u (body x vel)
    mterm += model.x['nu'][1]**2 * 0.001  # v (body y vel)
    mterm += model.x['nu'][3]**2 * 0.001  # r (yaw rate)

    # REMOVED COMPLETELY: Penalty for BCU_position at terminal state.


    # --- lterm (Running Cost) ---
    # Focus heavily on current depth (z) and zero vertical velocity (w)
    lterm = (model.x['eta'][2] - ref_eta[2])**2 * 100000  # Very high weight for depth (z)
    lterm += model.x['nu'][2]**2 * 1000 # Very high weight for zero vertical velocity (w) in running cost

    # Keep very small or zero weights for X, Y, Yaw, and their body velocities (u, v, r)
    lterm += (model.x['eta'][0] - ref_eta[0])**2 * 0.001  # x
    lterm += (model.x['eta'][1] - ref_eta[1])**2 * 0.001  # y
    lterm += (model.x['eta'][3] - ref_eta[3])**2 * 0.001  # yaw
    lterm += model.x['nu'][0]**2 * 0.001  # u (body x vel)
    lterm += model.x['nu'][1]**2 * 0.001  # v (body y vel)
    lterm += model.x['nu'][3]**2 * 0.001  # r (yaw rate)

    # REMOVED COMPLETELY: Penalty for BCU_position in running cost.

    mpc.set_objective(mterm=ca.sum1(mterm), lterm=lterm) # Ensure it's summed if you have multiple terms

    # --- Input Constraints (remain unchanged to keep T1-T4 at zero) ---
    mpc.bounds['lower','_u','T'] = 0.0
    mpc.bounds['upper','_u','T'] = 0.0
    mpc.bounds['lower','_u','BCU_INPUT'] = 50
    mpc.bounds['upper','_u','BCU_INPUT'] = 100

    # --- Uncertainty values (remain unchanged) ---
    mpc.set_uncertainty_values(
        k_rate = [0.25],
        mass = [19.16],
        Izz = [0.285],
        X_udot = [-18.0],
        Y_vdot = [-18.0],
        Z_wdot = [-25.0],
        N_rdot = [-0.5],
        d_lin_0 = [2.0],
        d_lin_1 = [2.0],
        d_lin_2 = [3.0],
        d_lin_3 = [0.1],
        d_quad_0 = [13.0],
        d_quad_1 = [13.0],
        d_quad_2 = [28.0],
        d_quad_3 = [0.1],
        d_ext = [ca.DM([0, 0, 0, 0]).reshape((4,1))] # activate d_ext if you want to use disturbances
    )

    mpc.setup()
    return mpc
