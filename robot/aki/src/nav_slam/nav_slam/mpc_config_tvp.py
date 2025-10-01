from casadi import *
import do_mpc
import numpy as np
import os
import sys




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
    v_max  = m.set_variable('parameter', 'v_max')

    # === Time-Varying Parameters (TVP) for Reference Tracking ===
    # These will be updated dynamically by the ROS node.
    m.set_variable(var_type='_tvp', var_name='x_ref')
    m.set_variable(var_type='_tvp', var_name='y_ref')
    m.set_variable(var_type='_tvp', var_name='z_ref')     # Target Z-position (from depth)
    m.set_variable(var_type='_tvp', var_name='yaw_ref')   # Target Yaw

    m.set_variable(var_type='_tvp', var_name='u_ref')     # Target body x velocity (usually 0)
    m.set_variable(var_type='_tvp', var_name='v_ref')     # Target body y velocity (usually 0)
    m.set_variable(var_type='_tvp', var_name='w_ref')     # Target body z velocity (usually 0)
    m.set_variable(var_type='_tvp', var_name='r_ref')     # Target yaw rate (usually 0)


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
    azimuths = [-np.pi/3, np.pi/3, np.pi]
    dirs = []

    for a in azimuths:
        dir_x = np.cos(a) * np.cos(theta_rad)
        dir_y = -np.sin(a) * np.cos(theta_rad) # negative sign because of slam global frame!
        dir_z = np.sin(theta_rad)  # upwards
        dirs.append([dir_x, dir_y, dir_z])

    # Direction for T4: purely in the negative Z direction
    dirs.append([0, 0, -1]) # [Fx, Fy, Fz] for T4 - 0 in X, 0 in Y, -1 in Z

    D_mat = DM(dirs).T  # shape (3,3)

    # Compute total body-frame force from thrusters
    f_xyz = D_mat @ T  # [Fx, Fy, Fz]

    m.set_expression(expr_name='f_xyz', expr=f_xyz)

    # Buoyancy control force Fz_x based on a (adjusted linearly between -1.6N and 1.6N)
    e = BCU_INPUT - BCU_position
    raw = k_rate * e
    BCU_dot = v_max * tanh(raw / v_max)

    Fz_x = (BCU_position - 75) * 0.0984 
    m.set_expression(expr_name='Fz_x_output', expr=Fz_x)
    
    # Thruster forces and torques
    tau = vertcat(
        f_xyz[0],
        f_xyz[1],
        f_xyz[2] + Fz_x,
        SX(0)  # No torque for roll and pitch since they're fixed
    )

    # === DYNAMICS ===
    nu_dot = inv(M) @ (tau - D_nu - C_nu + d_ext)

    # === RIGHT-HAND SIDES ===
    m.set_rhs('eta', eta_dot)
    m.set_rhs('nu', nu_dot)
    m.set_rhs('BCU_position', BCU_dot)

    m.setup()

    return m



import casadi as ca
def get_mpc(model, reference_goal):
    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 5, 
        't_step': 0.2, 
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 2,
        'store_full_solution': True,
        'n_robust': 1, 
        'nlpsol_opts': {
        'ipopt': {
            'linear_solver': 'ma27',
            'hsllib':'/usr/local/lib/libcoinhsl.so',
            'max_iter': 120  
        }
    }
    }

    mpc.set_param(**setup_mpc)

    mpc.n_horizon = setup_mpc['n_horizon']


    tvp_ref_eta = ca.vertcat(
        model.tvp['x_ref'],
        model.tvp['y_ref'],
        model.tvp['z_ref'], # Model's Z is upwards
        model.tvp['yaw_ref']
    )
    tvp_ref_nu = ca.vertcat(
        model.tvp['u_ref'],
        model.tvp['v_ref'],
        model.tvp['w_ref'], # Model's W (vertical velocity)
        model.tvp['r_ref']
    )
    
    # updated weights to match the desired behavior
    # thresholds (adapts if needeed)
    dx_thresh = 0.04   # 4 cm for XY
    dy_thresh = 0.04
    dz_thresh = 0.10   # 10 cm for Z

    # position errors
    ex = model.x['eta'][0] - tvp_ref_eta[0]
    ey = model.x['eta'][1] - tvp_ref_eta[1]
    ez = model.x['eta'][2] - tvp_ref_eta[2]

    # one-sided hinge penalties (no overshoot)
    pen_x = 0.5*((ca.fabs(ex)-dx_thresh) + ca.fabs(ca.fabs(ex)-dx_thresh))
    pen_y = 0.5*((ca.fabs(ey)-dy_thresh) + ca.fabs(ca.fabs(ey)-dy_thresh))
    pen_z = 0.5*((ca.fabs(ez)-dz_thresh) + ca.fabs(ca.fabs(ez)-dz_thresh))

    # weights (XY dominant)
    Wx, Wy, Wz = 2000.0, 2000.0, 400.0
    pos_cost = Wx*pen_x**2 + Wy*pen_y**2 + Wz*pen_z**2

    # velocity penalties
    Wu, Wv, Ww = 1200.0, 1200.0, 1800.0  
    vel_cost = Wu*(model.x['nu'][0]**2) \
         + Wv*(model.x['nu'][1]**2) \
         + Ww*(model.x['nu'][2]**2)


    # smoothness / effort
    thruster_cost = 10.0*ca.sum1(model.u['T']**2) 
    z_force_cost = 1e5*(model.aux['f_xyz'][2]**2)   # high weight so avoid thrust in Z direction

    # final objectives
    lterm = pos_cost + vel_cost + thruster_cost + z_force_cost
    mterm = pos_cost + vel_cost



    # Sum to scalar
    mpc.set_objective(mterm=ca.sum1(mterm), lterm=lterm)

    # mpc.set_rterm(T=50)

    # Input constraints
    # for pumps directly the Thrust, canbe switched later to match ros data
    mpc.bounds['lower','_u','T'] = 0.0      # Minimum Thrust per Thruster
    mpc.bounds['upper','_u','T'] = 5.58     # Maximum Thrust per Thruster -> calculating by finding the root
    # for the BCUs we constraint between 50 and 100, not the real forces, since a linear mapping exists
    mpc.bounds['lower','_u','BCU_INPUT'] = 50   # BCU force to move down
    mpc.bounds['upper','_u','BCU_INPUT'] = 100    # BCU force to move down

    # Model disturbances
    d_ext_min = ca.DM([-1.0, -1.0, -0.1, -0.3]).reshape((4,1))
    d_ext_max = ca.DM([1.0,  1.0,  0.1, 0.3]).reshape((4,1))
    d_ext_nominal = ca.DM([0, 0, 0, 0]).reshape((4,1))

    d_ext_uncertainty_scenarios = [
      d_ext_nominal,
      d_ext_min,
      d_ext_max
    ]

    
    mpc.set_uncertainty_values(
        k_rate = [0.45],
        v_max  = [6.86],
        mass = [20.085],
        Izz = [0.285],
        X_udot = [-4.3],  
        Y_vdot = [-4.3],
        Z_wdot = [-136.567],
        N_rdot = [-9.885],
        d_lin_0 = [10.01],  
        d_lin_1 = [10.01],
        d_lin_2 = [27.40],
        d_lin_3 = [0.1046],
        d_quad_0 = [62.13],
        d_quad_1 = [60.0],
        d_quad_2 = [1.9548],
        d_quad_3 = [3.888],
        d_ext = d_ext_uncertainty_scenarios
    )


    # === Configure Time-Varying Parameters (TVP) for Reference Tracking ===
    tvp_template = mpc.get_tvp_template()

    # Define the tvp_fun. This function will be called by do-mpc at each step.
    # It must return a dictionary where keys are the TVP names and values are
    # numpy arrays (or lists) with the values for each step of the horizon.
    # For a constant setpoint across the horizon, the value will be the same.
    def tvp_fun(t_now):
        # This function will be assigned in the ROS node's __init__ to access
        # the node's dynamically updated self.reference_goal.
        # For now, it uses the initial_reference_goal passed to get_mpc.
        for k in range(setup_mpc['n_horizon'] + 1): # +1 because horizon includes current time (t_0 to t_N)
            tvp_template['_tvp', k, 'x_ref'] = reference_goal['x']
            tvp_template['_tvp', k, 'y_ref'] = reference_goal['y']
            tvp_template['_tvp', k, 'z_ref'] = reference_goal['depth'] 
            tvp_template['_tvp', k, 'yaw_ref'] = reference_goal['yaw']
            tvp_template['_tvp', k, 'u_ref'] = 0.0 # Target zero velocity in body x
            tvp_template['_tvp', k, 'v_ref'] = 0.0 # Target zero velocity in body y
            tvp_template['_tvp', k, 'w_ref'] = 0.0 # Target zero velocity in body z
            tvp_template['_tvp', k, 'r_ref'] = 0.0 # Target zero yaw rate
        return tvp_template

    mpc.set_tvp_fun(tvp_fun) # Assign the initial TVP function



    mpc.setup()
    return mpc


def get_mpc_bcuonly(model, reference_goal):
    mpc = do_mpc.controller.MPC(model)


    setup_mpc = {
        'n_horizon': 30, 
        't_step': 0.5,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 2,
        'store_full_solution': True,
        'n_robust': 1, 
        'nlpsol_opts': {
        'ipopt': {
            'linear_solver': 'ma27',
            'hsllib':'/usr/local/lib/libcoinhsl.so',
            'max_iter': 100  # limit iterations to 100
        }
        }
    }

    mpc.set_param(**setup_mpc)

    mpc.n_horizon = setup_mpc['n_horizon']

    tvp_ref_eta = ca.vertcat(
        model.tvp['x_ref'],
        model.tvp['y_ref'],
        model.tvp['z_ref'], # Model's Z is upwards
        model.tvp['yaw_ref']
    )
    tvp_ref_nu = ca.vertcat(
        model.tvp['u_ref'],
        model.tvp['v_ref'],
        model.tvp['w_ref'], # Model's W (vertical velocity)
        model.tvp['r_ref']
    )


    # use scaled weights depending on usual errors
    Wz_L, Wz_T = 400.0, 4000.0 # cost for z coordinate for terminal and running cost
    Ww_L, Ww_T = 2500.0, 37500.0 # cost for w coordinate for terminal and running cost (velocity in z)
    Wtiny      = 1e-4   # or 0 if you also add explicit bounds

    # --- mterm (Terminal Cost) ---
    # Focus heavily on final depth (z) and zero vertical velocity (w)
    mterm = (model.x['eta'][2] - tvp_ref_eta[2])**2 * Wz_T  # Very high weight for depth (z)
    mterm += (model.x['nu'][2]- tvp_ref_nu[2])**2 * Ww_T # Very high weight for zero vertical velocity (w) at terminal state

    # Keep very small or zero weights for X, Y, Yaw, and their body velocities (u, v, r)
    mterm += (model.x['eta'][0] - tvp_ref_eta[0])**2 * Wtiny  # x
    mterm += (model.x['eta'][1] - tvp_ref_eta[1])**2 * Wtiny  # y
    mterm += (model.x['eta'][3] - tvp_ref_eta[3])**2 * Wtiny  # yaw
    mterm += model.x['nu'][0]**2 * Wtiny  # u (body x vel)
    mterm += model.x['nu'][1]**2 * Wtiny  # v (body y vel)
    mterm += model.x['nu'][3]**2 * Wtiny  # r (yaw rate)

    # --- lterm (Running Cost) ---
    # Focus heavily on current depth (z) and zero vertical velocity (w)
    lterm = (model.x['eta'][2] - tvp_ref_eta[2])**2 * Wz_L  # Very high weight for depth (z)
    lterm += model.x['nu'][2]**2 * Ww_L # Very high weight for zero vertical velocity (w) in running cost

    # Keep very small or zero weights for X, Y, Yaw, and their body velocities (u, v, r)
    lterm += (model.x['eta'][0] - tvp_ref_eta[0])**2 * Wtiny  # x
    lterm += (model.x['eta'][1] - tvp_ref_eta[1])**2 * Wtiny  # y
    lterm += (model.x['eta'][3] - tvp_ref_eta[3])**2 * Wtiny  # yaw
    lterm += model.x['nu'][0]**2 * Wtiny  # u (body x vel)
    lterm += model.x['nu'][1]**2 * Wtiny  # v (body y vel)
    lterm += model.x['nu'][3]**2 * Wtiny  # r (yaw rate)
    lterm += (model.u['BCU_INPUT']-75.0)**2 * 0.01  # Small penalty for BCU input to avoid large changes


    mpc.set_objective(mterm=ca.sum1(mterm), lterm=lterm) # Ensure it's summed if you have multiple terms


    # --- Input Constraints (remain unchanged to keep T1-T4 at zero) ---
    mpc.bounds['lower','_u','T'] = 0.0
    mpc.bounds['upper','_u','T'] = 0.0
    mpc.bounds['lower','_u','BCU_INPUT'] = 50
    mpc.bounds['upper','_u','BCU_INPUT'] = 100

    # --- Uncertainty values (remain unchanged) ---

    mpc.set_uncertainty_values(
        k_rate = [0.45],
        v_max  = [6.86],
        mass = [20.085],
        Izz = [0.285],
        X_udot = [-4.3],  
        Y_vdot = [-4.3],
        Z_wdot = [-136.567],
        N_rdot = [-9.885],
        d_lin_0 = [10.01],  
        d_lin_1 = [10.01],
        d_lin_2 = [27.40],
        d_lin_3 = [0.1046],
        d_quad_0 = [62.13],
        d_quad_1 = [60.0],
        d_quad_2 = [1.9548],
        d_quad_3 = [3.888],
        d_ext = [ca.DM([0, 0, 0, 0]).reshape((4,1))]
    )

     # === Configure Time-Varying Parameters (TVP) for Reference Tracking ===
    tvp_template = mpc.get_tvp_template()

    # Define the tvp_fun. This function will be called by do-mpc at each step.
    # It must return a dictionary where keys are the TVP names and values are
    # numpy arrays (or lists) with the values for each step of the horizon.
    # For a constant setpoint across the horizon, the value will be the same.
    def tvp_fun(t_now):
        # This function will be assigned in the ROS node's __init__ to access
        # the node's dynamically updated self.reference_goal.
        # For now, it uses the initial_reference_goal passed to get_mpc.
        for k in range(setup_mpc['n_horizon'] + 1): # +1 because horizon includes current time (t_0 to t_N)
            tvp_template['_tvp', k, 'x_ref'] = reference_goal['x']
            tvp_template['_tvp', k, 'y_ref'] = reference_goal['y']
            tvp_template['_tvp', k, 'z_ref'] = reference_goal['depth'] 
            tvp_template['_tvp', k, 'yaw_ref'] = reference_goal['yaw']
            tvp_template['_tvp', k, 'u_ref'] = 0.0 # Target zero velocity in body x
            tvp_template['_tvp', k, 'v_ref'] = 0.0 # Target zero velocity in body y
            tvp_template['_tvp', k, 'w_ref'] = 0.0 # Target zero velocity in body z
            tvp_template['_tvp', k, 'r_ref'] = 0.0 # Target zero yaw rate
        return tvp_template

    mpc.set_tvp_fun(tvp_fun) # Assign the initial TVP function



    mpc.setup()
    return mpc
