
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
    v_max  = m.set_variable('parameter', 'v_max')

    # === Time-Varying Parameters (TVP) for Reference Tracking ===
    # These will be updated dynamically by your ROS node.
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
    # azimuths = [0, 2*np.pi/3, 4*np.pi/3]
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
    # BCU_dot = k_rate * (BCU_INPUT - BCU_position)  # Change in position based on setpoint a and current position
    e = BCU_INPUT - BCU_position
    raw = k_rate * e
    # use CasADi’s if_else to saturate
    #BCU_dot = if_else(fabs(raw) < v_max,raw, v_max * sign(e))
    BCU_dot = v_max * tanh(raw / v_max)

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
        'n_horizon': 5, # 2 seconds in future is absoluetly fine, everything else doesn£t help
        't_step': 0.2, 
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 2,
        'store_full_solution': True,
        'n_robust': 1, # use some other values to have uncertainty parameters inside the model
        'nlpsol_opts': {
        'ipopt': {
            'linear_solver': 'ma57',
            'hsllib':'/usr/local/lib/libcoinhsl.so',
            'max_iter': 120  # limit iterations to 100
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
    # mterm = (model.x['eta'] - tvp_ref_eta)**2 + (model.x['nu'] - tvp_ref_nu)**2


    # Define reference values for eta and nu
    #ref_eta = ca.DM([5, 4, -3, 0]).reshape((4, 1))
    #ref_nu  = ca.DM([0, 0, 0, 0]).reshape((4, 1))
    
    # If you want to use a specific reference goal, uncomment the following lines:
    # ref_eta = ca.DM([reference_goal['x'], reference_goal['y'], reference_goal['depth'], reference_goal['yaw']]).reshape((4, 1))
    # ref_nu  = ca.DM([0.0, 0.0, 0.0, 0.0]).reshape((4, 1))


    # Define terminal and stage costs
    # mterm = (model.x['eta'] - ref_eta)**2 + (model.x['nu'] - ref_nu)**2

    # --- Penalties --- approach from simulation
    # Z_FORCE_PENALTY_WEIGHT = 50.0 # High penalty for Z-force from thrusters (adjust as needed)
    # THRUSTER_USE_PENALTY_WEIGHT = 0.01 # General penalty for thruster usage
    # W_NU_Z = 5 # penalizes speed of robot, should move down slow to avoid overshoot due to high inertia
    #lterm = ca.sum1(mterm) + THRUSTER_USE_PENALTY_WEIGHT * ca.sum1(model.u['T']**2) + 0.01 * ca.sum1(model.u['BCU_INPUT']**2) + Z_FORCE_PENALTY_WEIGHT * (model.aux['f_xyz'][2]**2)
    # not penalize BCU, makes no sense, outmax is nice. Extended BCU is same as retracted -> maybe nice to be closer to neutrally buoyant, but then transform to forces!
    # lterm = ca.sum1(mterm) + THRUSTER_USE_PENALTY_WEIGHT * ca.sum1(model.u['T']**2) + Z_FORCE_PENALTY_WEIGHT * (model.aux['f_xyz'][2]**2) + W_NU_Z * (model.x['nu'][2]**2)

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
    Wu, Wv, Ww = 1200.0, 1200.0, 1800.0   # tuneable: w heavier if Z is trickier
    vel_cost = Wu*(model.x['nu'][0]**2) \
         + Wv*(model.x['nu'][1]**2) \
         + Ww*(model.x['nu'][2]**2)


    # smoothness / effort
    thruster_cost = 10.0*ca.sum1(model.u['T']**2) # 10.0 is better than rterm 50
    z_force_cost = 1e5*(model.aux['f_xyz'][2]**2)   # high weight so avoid thrust in Z direction

    # final objectives
    lterm = pos_cost + vel_cost + thruster_cost + z_force_cost
    # lterm = pos_cost + vel_cost + z_force_cost
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
    #mpc.bounds['lower', '_x', 'eta'][4] = -np.deg2rad(80)
    #mpc.bounds['upper', '_x', 'eta'][4] =  np.deg2rad(80)

    # Model disturbances
    d_ext_min = ca.DM([-1.0, -1.0, -0.1, -0.3]).reshape((4,1))
    d_ext_max = ca.DM([1.0,  1.0,  0.1, 0.3]).reshape((4,1))
    d_ext_nominal = ca.DM([0, 0, 0, 0]).reshape((4,1))

    d_ext_uncertainty_scenarios = [
      d_ext_nominal,
      d_ext_min,
      d_ext_max
    ]

    # mpc.set_uncertainty_values(
    #     #k_rate = [0.25],
    #     k_rate = [2.5],
    #     v_max  = [6.861063465],
    #     mass = [19.16],
    #     Izz = [0.285],
    #     X_udot = [-4.317],  # Example uncertainty scenarios
    #     Y_vdot = [-3.837],
    #     Z_wdot = [-8.959],
    #     N_rdot = [-9.885],
    #     d_lin_0 = [4.042],  # Example uncertainty scenarios
    #     d_lin_1 = [3.926],
    #     d_lin_2 = [4.022],
    #     d_lin_3 = [0.1046],
    #     d_quad_0 = [4.09],
    #     d_quad_1 = [4.071],
    #     d_quad_2 = [4.084],
    #     d_quad_3 = [3.888],
    #     d_ext = d_ext_uncertainty_scenarios
    # )
    
    mpc.set_uncertainty_values(
        #k_rate = [0.25],
        k_rate = [2.5],
        v_max  = [6.989],
        mass = [20.085],
        Izz = [0.285],
        X_udot = [-4.0],  # Example uncertainty scenarios
        Y_vdot = [-4.0],
        Z_wdot = [-136.567],
        N_rdot = [-9.885],
        d_lin_0 = [10.01],  # Example uncertainty scenarios
        d_lin_1 = [10.01],
        d_lin_2 = [27.40],
        d_lin_3 = [0.1046],
        d_quad_0 = [60.0],
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
        # This function will be assigned in your ROS node's __init__ to access
        # the node's dynamically updated self.reference_goal.
        # For now, it uses the initial_reference_goal passed to get_mpc.
        for k in range(setup_mpc['n_horizon'] + 1): # +1 because horizon includes current time (t_0 to t_N)
            tvp_template['_tvp', k, 'x_ref'] = reference_goal['x']
            tvp_template['_tvp', k, 'y_ref'] = reference_goal['y']
            tvp_template['_tvp', k, 'z_ref'] = reference_goal['depth'] # Make sure this matches your model's Z-axis (upwards positive)
            tvp_template['_tvp', k, 'yaw_ref'] = reference_goal['yaw']
            tvp_template['_tvp', k, 'u_ref'] = 0.0 # Target zero velocity in body x
            tvp_template['_tvp', k, 'v_ref'] = 0.0 # Target zero velocity in body y
            tvp_template['_tvp', k, 'w_ref'] = 0.0 # Target zero velocity in body z
            tvp_template['_tvp', k, 'r_ref'] = 0.0 # Target zero yaw rate
            # If you add bcu_pos_ref to model tvp and need to pass it:
            # tvp_template['_tvp', k, 'bcu_pos_ref'] = initial_reference_goal['bcu_pos']
        return tvp_template

    mpc.set_tvp_fun(tvp_fun) # Assign the initial TVP function



    mpc.setup()
    return mpc


def get_mpc_bcuonly(model, reference_goal):
    mpc = do_mpc.controller.MPC(model)


    setup_mpc = {
        'n_horizon': 30, # 15 seconds in future, large terminal cost is fine, it is realistic to reach the goal within this period of time
        't_step': 0.5,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 2,
        'store_full_solution': True,
        'n_robust': 1, # use some other values to have uncertainty parameters inside the model
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

    # ref_eta = ca.DM([reference_goal['x'], reference_goal['y'], reference_goal['depth'], reference_goal['yaw']]).reshape((4, 1))
    # ref_nu  = ca.DM([0.0, 0.0, 0.0, 0.0]).reshape((4, 1))

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

    # REMOVED COMPLETELY: Penalty for BCU_position at terminal state.


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

    # Add one-sided no-crossing soft contraints to avoid overshoot


    # # Residual that becomes positive only when we'd overshoot:
    # # r_k = max(0, dir_no_cross * (z_ref - z))
    # overshoot_residual = 0.5 * ((tvp_ref_eta[2] - model.x['eta'][2]) \
    #                             + ca.fabs((tvp_ref_eta[2] - model.x['eta'][2])))

    # # Soft constraint via heavy penalty in running cost:
    # Wovershoot = 1e5   # big; raise if you *ever* see a crossing
    # lterm += Wovershoot * overshoot_residual**2


    # REMOVED COMPLETELY: Penalty for BCU_position in running cost.

    mpc.set_objective(mterm=ca.sum1(mterm), lterm=lterm) # Ensure it's summed if you have multiple terms

    # mpc.set_rterm(BCU_INPUT=1e-2)

    # --- Input Constraints (remain unchanged to keep T1-T4 at zero) ---
    mpc.bounds['lower','_u','T'] = 0.0
    mpc.bounds['upper','_u','T'] = 0.0
    mpc.bounds['lower','_u','BCU_INPUT'] = 50
    mpc.bounds['upper','_u','BCU_INPUT'] = 100

    # --- Uncertainty values (remain unchanged) ---

    # mpc.set_uncertainty_values(
    #     k_rate = [0.25],
    #     mass = [19.16],
    #     Izz = [0.285],
    #     X_udot = [-4.317],
    #     Y_vdot = [-3.837],
    #     Z_wdot = [-8.959],
    #     N_rdot = [-9.885],
    #     d_lin_0 = [4.042],
    #     d_lin_1 = [3.926],
    #     d_lin_2 = [4.022],
    #     d_lin_3 = [0.1046],
    #     d_quad_0 = [4.09],
    #     d_quad_1 = [4.071],
    #     d_quad_2 = [4.084],
    #     d_quad_3 = [3.888],
    #     d_ext = [ca.DM([0, 0, 0, 0]).reshape((4,1))] # activate d_ext if you want to use disturbances

    # )
    # d_ext_min = ca.DM([0, 0,-0.1, 0]).reshape((4,1))
    # d_ext_max = ca.DM([0,  0,  1.0, 0]).reshape((4,1))
    # d_ext_nominal = ca.DM([0, 0, 0, 0]).reshape((4,1))

    # d_ext_uncertainty_scenarios = [
    #   d_ext_nominal,
    #   d_ext_min,
    #   d_ext_max
    # ]

    mpc.set_uncertainty_values(
        #k_rate = [0.25],
        k_rate = [2.5],
        v_max  = [6.989],
        mass = [20.085],
        Izz = [0.285],
        X_udot = [-4.0],  # Example uncertainty scenarios
        Y_vdot = [-4.0],
        Z_wdot = [-136.567],
        N_rdot = [-9.885],
        d_lin_0 = [10.01],  # Example uncertainty scenarios
        d_lin_1 = [10.01],
        d_lin_2 = [27.40],
        d_lin_3 = [0.1046],
        d_quad_0 = [60.0],
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
        # This function will be assigned in your ROS node's __init__ to access
        # the node's dynamically updated self.reference_goal.
        # For now, it uses the initial_reference_goal passed to get_mpc.
        for k in range(setup_mpc['n_horizon'] + 1): # +1 because horizon includes current time (t_0 to t_N)
            tvp_template['_tvp', k, 'x_ref'] = reference_goal['x']
            tvp_template['_tvp', k, 'y_ref'] = reference_goal['y']
            tvp_template['_tvp', k, 'z_ref'] = reference_goal['depth'] # Make sure this matches your model's Z-axis (upwards positive)
            tvp_template['_tvp', k, 'yaw_ref'] = reference_goal['yaw']
            tvp_template['_tvp', k, 'u_ref'] = 0.0 # Target zero velocity in body x
            tvp_template['_tvp', k, 'v_ref'] = 0.0 # Target zero velocity in body y
            tvp_template['_tvp', k, 'w_ref'] = 0.0 # Target zero velocity in body z
            tvp_template['_tvp', k, 'r_ref'] = 0.0 # Target zero yaw rate
            # If you add bcu_pos_ref to model tvp and need to pass it:
            # tvp_template['_tvp', k, 'bcu_pos_ref'] = initial_reference_goal['bcu_pos']
        return tvp_template

    mpc.set_tvp_fun(tvp_fun) # Assign the initial TVP function



    mpc.setup()
    return mpc
