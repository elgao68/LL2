/////////////////////////////////////////////////////////////////////////////
//
//  _test_ode_int.h
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <_test_ode_int.h>

void test_ode_int() {

	/////////////////////////////////////////////////////////////////////////////////////
	// FIRMWARE/CONTROL PARAMETERS - GAO:
	/////////////////////////////////////////////////////////////////////////////////////

	// These are only needed for data logging (set_LL_exercise_feedback_help()):
	lowerlimb_mech_readings_t   LL_mech_readings;
	lowerlimb_motors_settings_t LL_motors_settings;

	traj_ctrl_params_t          traj_ctrl_params;
	admitt_model_params_t       admitt_model_params;
	lowerlimb_ref_kinematics_t	ref_kinematics;

	/////////////////////////////////////////////////////////////////////////////////////
	// Sensor variables:
	/////////////////////////////////////////////////////////////////////////////////////

	// End-effector force measurements:
	double F_end_m[N_COORD_2D];

	/////////////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - REFERENCE:
	/////////////////////////////////////////////////////////////////////////////////////

	// Integrator time step:
	double dt_k = 0.001; // (double)DT_STEP_MSEC/MSEC_PER_SEC; // integrator step (initial)

	// Reference position and velocity:
	double    p_ref[N_COORD_2D] = {0.0, 0.0};
	double dt_p_ref[N_COORD_2D] = {0.0, 0.0};

	// Cycle phase and instantaneous frequency:
	double    phi_ref = 0.0;
	double dt_phi_ref = 0.0;

	// Trajectory path tangent vector:
	double u_t_ref[N_COORD_2D] = {0.0, 0.0};

	/////////////////////////////////////////////////////////////////////////////////////
	// Counters and timers:
	/////////////////////////////////////////////////////////////////////////////////////

	double T_RUN_MAX;
	int step_i = 0;
	double t_ref;

	/////////////////////////////////////////////////////////////////////////////////////
	// Trajectory parameters:
	/////////////////////////////////////////////////////////////////////////////////////

	traj_ctrl_params.cycle_period   = 8.0;
	traj_ctrl_params.exp_blend_time = 4.0;
	traj_ctrl_params.semiaxis_x     = 0.15;
	traj_ctrl_params.semiaxis_y     = 0.1;
	traj_ctrl_params.rot_angle      = 0;
	traj_ctrl_params.cycle_dir      = 1;

	/////////////////////////////////////////////////////////////////////////////////////
	// Admittance model:
	/////////////////////////////////////////////////////////////////////////////////////

	static double damp_ratio = 0.2;
	static double w_n        = 2*PI*1.0; // natural frequency
	static double sigma;

	admitt_model_params.inertia_x = admitt_model_params.inertia_y = 10.0;
	admitt_model_params.stiffness = admitt_model_params.inertia_x*w_n*w_n;

	admitt_model_params.damping =
			2*damp_ratio*
			sqrt(admitt_model_params.stiffness*admitt_model_params.inertia_x);

	sigma = damp_ratio*w_n;
	T_RUN_MAX = 3.0/sigma;

	admitt_model_params.p_eq_x    = 0;
	admitt_model_params.p_eq_y    = 0;
	admitt_model_params.Fx_offset = 0;
	admitt_model_params.Fy_offset = 0;

	do {
		// Apply delay:
		HAL_Delay(25);

		/////////////////////////////////////////////////////////////////////////////////////
		// Extract measured end-effector forces:
		/////////////////////////////////////////////////////////////////////////////////////

		F_end_m[IDX_X] = 0;
		F_end_m[IDX_Y] = 0;

		/////////////////////////////////////////////////////////////////////////////////////
		// Motor algorithm computation:
		/////////////////////////////////////////////////////////////////////////////////////

		traj_reference_step_active(p_ref, dt_p_ref, &phi_ref, &dt_phi_ref, u_t_ref, dt_k,
				F_end_m,
				traj_ctrl_params, admitt_model_params, USE_ADMITT_MODEL_CONSTR);

		// Set reference kinematics struct:
		for (int c_i = 0; c_i < N_COORD_2D; c_i++) {
			ref_kinematics.p_ref[c_i]    = p_ref[c_i];
			ref_kinematics.dt_p_ref[c_i] = dt_p_ref[c_i];
		}

		ref_kinematics.phi_ref    = phi_ref;
		ref_kinematics.dt_phi_ref = dt_phi_ref;

		/////////////////////////////////////////////////////////////////////////////////////
		// Distal Force Sensor - Change only when updating TCP Protocol
		// Input Brakes info from TCP System Info
		/////////////////////////////////////////////////////////////////////////////////////

		set_LL_exercise_feedback_help(getUpTime(), &LL_mech_readings, &LL_motors_settings, &ref_kinematics);

		/////////////////////////////////////////////////////////////////////////////////////
		// Update step time:
		/////////////////////////////////////////////////////////////////////////////////////

		t_ref = dt_k*step_i;

		// ITM console output:

		/*
		if (step_i % (DT_DISP_MSEC_REALTIME/(int)(1000*dt_k)) == 0)
			printf("%d\t%f\t(%d)\t%f\t%f\t%f\t%f\t%f\t%f\n",
				step_i,
				t_ref,
				(int)(up_time_end - up_time),
				phi_ref,
				dt_phi_ref,
				p_ref[IDX_X],
				p_ref[IDX_Y],
				dt_p_ref[IDX_X],
				dt_p_ref[IDX_Y]);
		 */
	} while (t_ref <= T_RUN_MAX);
}
