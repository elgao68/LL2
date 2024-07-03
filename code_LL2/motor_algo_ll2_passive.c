/////////////////////////////////////////////////////////////////////////////
//
// motor_algo_ll2_passive.c
//
// Created on: 2024.06.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "motor_algo_ll2_multi.h"

///////////////////////////////////////////////////////////////////////////////
// MOTION ALGORITHMS - ELLIPTICAL TRAJECTORY - PASSIVE:
///////////////////////////////////////////////////////////////////////////////

void
traj_ref_step_passive_elliptic(
		double p_ref[],	double dt_p_ref[],
		double* phi_ref, double* dt_phi_ref,
		double u_t_ref[], double dt_k,
		traj_ctrl_params_t traj_ctrl_params, int8_t switch_traj, int8_t use_traj_params_variable) {

	///////////////////////////////////////////////////////////////////////////////
	// COPY TRAJECTORY PARAMETERS TO LOCAL SCOPE VARIABLES:
	///////////////////////////////////////////////////////////////////////////////

	static double T_cycle;
	static double T_exp;
	static double ax_x;
	static double ax_y;
	static double rot_ang;
	static int cycle_dir;
	static double sig_exp;

	static double ax_x_adj;
	static double ax_y_adj;

	static double frac_dt_phi = 1.0;

	static uint8_t initial = 1;

	if (initial) {
		T_cycle = traj_ctrl_params.cycle_period;
		T_exp   = traj_ctrl_params.exp_blend_time;
		ax_x    = traj_ctrl_params.semiaxis_x;
		ax_y    = traj_ctrl_params.semiaxis_y;
		rot_ang  = traj_ctrl_params.rot_angle;
		cycle_dir  = (int)traj_ctrl_params.cycle_dir;
		sig_exp = 3.0/T_exp;

		if (use_traj_params_variable) {
			ax_x_adj = 0;
			ax_y_adj = 0;
		}
		else {
			ax_x_adj = ax_x;
			ax_y_adj = ax_y;
		}

		initial = 0;
	}

	///////////////////////////////////////////////////////////////////////////////
	// Set up timers & behaviors:
	///////////////////////////////////////////////////////////////////////////////

	static int step_int = 0; // algorithm step counter
	double t_ref = dt_k*step_int;

	///////////////////////////////////////////////////////////////////////////////
	// Variable parameters - behaviors:
	///////////////////////////////////////////////////////////////////////////////

	static int8_t traj_params_behav = TRAJ_PARAMS_STEADY;
	static double t_param_o = 0;

	if (use_traj_params_variable)  {
		static double ax_x_o;
		static double ax_y_o;

		///////////////////////////////////////////////////////////////////////////////
		// Trajectory start and stop: reference values
		///////////////////////////////////////////////////////////////////////////////

		if (switch_traj == SWITCH_TRAJ_START && traj_params_behav != TRAJ_PARAMS_GROW)	{
			traj_params_behav = TRAJ_PARAMS_GROW;

			t_param_o = t_ref;
			ax_x_o = ax_x_adj;
			ax_y_o = ax_y_adj;

		}
		else if (switch_traj == SWITCH_TRAJ_END && traj_params_behav != TRAJ_PARAMS_DECAY) {
			traj_params_behav = TRAJ_PARAMS_DECAY;

			t_param_o = t_ref;
			ax_x_o = ax_x_adj;
			ax_y_o = ax_y_adj;
		}

		///////////////////////////////////////////////////////////////////////////////
		//  Adjusted trajectory path parameters:
		///////////////////////////////////////////////////////////////////////////////

		if (traj_params_behav == TRAJ_PARAMS_GROW) {
			ax_x_adj = (1.0 - exp(-sig_exp*(t_ref - t_param_o)))*(ax_x - ax_x_o) + ax_x_o;
			ax_y_adj = (1.0 - exp(-sig_exp*(t_ref - t_param_o)))*(ax_y - ax_y_o) + ax_y_o;
		}
		else if (traj_params_behav == TRAJ_PARAMS_DECAY) {
			ax_x_adj = exp(-sig_exp*(t_ref - t_param_o))*ax_x_o;
			ax_y_adj = exp(-sig_exp*(t_ref - t_param_o))*ax_y_o;
		}
	}

	///////////////////////////////////////////////////////////////////////////////
	// Fixed parameters:
	///////////////////////////////////////////////////////////////////////////////

	else {
		if (switch_traj == SWITCH_TRAJ_END && traj_params_behav != TRAJ_PARAMS_DECAY) {
			traj_params_behav = TRAJ_PARAMS_DECAY;

			t_param_o = t_ref;
		}

		///////////////////////////////////////////////////////////////////////////////
		// Make frequency decay:
		///////////////////////////////////////////////////////////////////////////////

		if (traj_params_behav == TRAJ_PARAMS_DECAY)
			frac_dt_phi = exp(-sig_exp*(t_ref - t_param_o));
	}

	///////////////////////////////////////////////////////////////////////////////
	// ITM console output:
	///////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_OUT_TRAJ_REF
		if (step_int == 0) {
			printf("\n");
			printf("   traj_ref_step_passive_elliptic(): \n");
			printf("   T_cycle = %f\n", T_cycle);
			printf("   T_exp   = %f\n", T_exp);
			printf("   ax_x    = %f\n", ax_x);
			printf("   ax_y    = %f\n", ax_y);
			printf("   rot_ang  = %f\n", rot_ang);
			printf("   cycle_dir  = %f\n", (double)cycle_dir);
			printf("\n");
		}
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Obtain reference trajectory position and velocity:
	///////////////////////////////////////////////////////////////////////////////

	// Compute instantaneous frequency and phase:
	*dt_phi_ref = frac_dt_phi*2*PI/T_cycle;
	*phi_ref    = (*dt_phi_ref)*t_ref;

	// Compute reference trajectory position and velocity from PHASE:
	traj_ellipse_points(*phi_ref, *dt_phi_ref, p_ref, dt_p_ref, u_t_ref,
		ax_x_adj, ax_y_adj, rot_ang);

	///////////////////////////////////////////////////////////////////////////////
	// Increase step counter:
	///////////////////////////////////////////////////////////////////////////////

	step_int++;
}

///////////////////////////////////////////////////////////////////////////////
// MOTION ALGORITHMS - ISOMETRIC "TRAJECTORY":
///////////////////////////////////////////////////////////////////////////////

void traj_ref_step_isometric(
		double p_ref[],	double dt_p_ref[],
		double* phi_ref, double* dt_phi_ref,
		double u_t_ref[]) {

	for (int c_i = IDX_X; c_i <= IDX_Y; c_i++) {
		p_ref[c_i]    = 0.0;
		dt_p_ref[c_i] = 0.0;
		u_t_ref[c_i]  = 0.0;
	}

	*phi_ref    = 0.0;
	*dt_phi_ref = 0.0;
}
