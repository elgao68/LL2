/////////////////////////////////////////////////////////////////////////////
//
// motor_algo_ll2_passive.c
//
// Created on: 2024.06.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <motor_algo_ll2.h>

///////////////////////////////////////////////////////////////////////////////
// ELLIPTICAL TRAJECTORY - PASSIVE:
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
// ISOMETRIC "TRAJECTORY":
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

///////////////////////////////////////////////////////////////////////////////
// CALIBRATION TRAJECTORY:
///////////////////////////////////////////////////////////////////////////////

void
traj_ref_calibration_ll2(
	double p_ref[], double dt_p_ref[], uint8_t* calib_enc_on, calib_traj_t* calib_traj, uint8_t* idx_scale, double z_intern_o_dbl[],
	double dt_k, double p_m[], double dt_p_m[], double phi_o, double dt_phi_o,
	lowerlimb_motors_settings_t* LL_motors_settings, traj_ctrl_params_t* traj_ctrl_params, uint8_t traj_exerc_type, double v_calib, double frac_ramp_calib) {

	// Switching variables:
	static uint8_t init_calib_traj = 0;
	static uint8_t calib_traj_prev = CalibTraj_Null;

	const double ERR_POS_REL = 0.005; // HACK: we specify a minimum relative displacement before checking for contact conditions

	// Calibration end points:
	static double p_calib_o[N_COORD_2D] = {0.0, 0.0};
	static double p_calib_f[N_COORD_2D] = {0.0, 0.0};

	// Timers:
	static uint16_t step_i  = 0;
	static double T_f_calib = 0.0;

	static double t_calib   = 0.0;
	t_calib = step_i*dt_k;

	// Dummy variables:
	double u_t_ref_dum[N_COORD_2D] = {0.0, 0.0};
	double pos_rel_calib_dum       = 0.0;
	double dt_pos_rel_calib_dum    = 0.0;

	///////////////////////////////////////////////////////////////////////////////
	// Test for initial and switching conditions:
	///////////////////////////////////////////////////////////////////////////////

	if (*calib_traj == CalibTraj_Null) {

		// Set up next calibration trajectory:
		*calib_traj = CalibTraj_1_Y_Travel;

		p_calib_o[IDX_X] = p_m[IDX_X];
		p_calib_o[IDX_Y] = p_m[IDX_Y];

		p_calib_f[IDX_X] = p_calib_o[IDX_X];
		p_calib_f[IDX_Y] = p_calib_o[IDX_Y] - DIST_CALIB_MAX;

		// Control gains scale array:
		*idx_scale = IDX_SCALE_CALIB;

		// Trajectory initiation command:
		init_calib_traj  = 1;
	}

	else if (
		*calib_traj == CalibTraj_1_Y_Travel &&
		fabs(dt_p_m[IDX_Y]) < THR_DT_P_CONTACT &&
		fabs(LL_motors_settings->left.volt)  > THR_VOLTAGE_CONTACT &&
		fabs(LL_motors_settings->right.volt) > THR_VOLTAGE_CONTACT &&
		norm2_2d(p_m[IDX_X] - p_calib_o[IDX_X], p_m[IDX_Y] - p_calib_o[IDX_Y]) > ERR_POS_REL) {

		// Set up next calibration trajectory:
		*calib_traj = CalibTraj_2_X_Travel;

		p_calib_o[IDX_X] = p_m[IDX_X];
		p_calib_o[IDX_Y] = p_m[IDX_Y];

		p_calib_f[IDX_X] = p_calib_o[IDX_X] - DIST_CALIB_MAX;
		p_calib_f[IDX_Y] = p_calib_o[IDX_Y];

		// Control gains scale array:
		*idx_scale = IDX_SCALE_CALIB;

		// Trajectory initiation command:
		init_calib_traj  = 1;
	}

	else if (
		*calib_traj == CalibTraj_2_X_Travel &&
		fabs(dt_p_m[IDX_X]) < THR_DT_P_CONTACT &&
		fabs(LL_motors_settings->left.volt)  > THR_VOLTAGE_CONTACT &&
		fabs(LL_motors_settings->right.volt) > THR_VOLTAGE_CONTACT &&
		norm2_2d(p_m[IDX_X] - p_calib_o[IDX_X], p_m[IDX_Y] - p_calib_o[IDX_Y]) > ERR_POS_REL) {

		// Reset encoders - CRITICAL:
		qei_count_L_reset();
		qei_count_R_reset();

		// Set up next calibration trajectory:
		*calib_traj = CalibTraj_3_Travel_to_ORG;

		p_calib_o[IDX_X] = 0;
		p_calib_o[IDX_Y] = 0;

		p_calib_f[IDX_X] = 0.5*D_WKSPC_LL2_X;
		p_calib_f[IDX_Y] = 0.5*D_WKSPC_LL2_Y;

		// Control gains scale array:
		*idx_scale = IDX_SCALE_EXERCISE;

		// Trajectory initiation command:
		init_calib_traj  = 1;
	}

	else if (
		*calib_traj == CalibTraj_3_Travel_to_ORG &&
		t_calib >= T_f_calib) {

		/*
		#if USE_ITM_OUT_CALIB_CHECK
			printf("   ----------------------------\n");
			printf("   CalibTraj_3_Travel_to_ORG: \n");
			printf("   calib_traj (%d)\t= [%s] \n", *calib_traj, CALIB_TRAJ_STR[*calib_traj]);
			printf("   calib_enc_on\t\t= [%d] \n", *calib_enc_on);
			printf("   t_calib (%d)\t= [%3.2f], T_f_calib = [%3.2f] \n\n", step_i, t_calib, T_f_calib);
		#endif
		*/

		// Reset encoders - CRITICAL:
		qei_count_L_reset();
		qei_count_R_reset();

		// Set up next calibration trajectory:
		*calib_traj = CalibTraj_4_Travel_to_P_Start_Exe;

		p_calib_o[IDX_X] = 0;
		p_calib_o[IDX_Y] = 0;

		if (traj_exerc_type == EllipticTraj || traj_exerc_type == LinearTraj) {
			// CALIBRATION: this will only work with the TRAJ_PARAMS_VARIABLE_OFF option in (LL_sys_info.exercise_state == RUNNING):
			traj_ellipse_points(phi_o, dt_phi_o, p_ref, dt_p_ref, u_t_ref_dum,
				traj_ctrl_params->semiaxis_x, traj_ctrl_params->semiaxis_y, traj_ctrl_params->rot_angle);

			p_calib_f[IDX_X] = p_ref[IDX_X];
			p_calib_f[IDX_Y] = p_ref[IDX_Y];
		}
		else {
			// Safety catch:
			p_calib_f[IDX_X] = 0;
			p_calib_f[IDX_Y] = 0;
		}

		// Active trajectory control: create initial condition for internal state (CRITICAL)
		z_intern_o_dbl[IDX_X  ]    = p_ref[IDX_X];
		z_intern_o_dbl[IDX_Y  ]    = p_ref[IDX_Y];
		z_intern_o_dbl[IDX_PHI]    = phi_o;

		z_intern_o_dbl[IDX_DT_X  ] = 0;
		z_intern_o_dbl[IDX_DT_Y  ] = 0;
		z_intern_o_dbl[IDX_DT_PHI] = dt_phi_o;

		// Control gains scale array:
		*idx_scale = IDX_SCALE_EXERCISE;

		// Trajectory initiation command:
		init_calib_traj  = 1;
	}

	// Encoders calibration - exit condition:
	else if (
		*calib_traj == CalibTraj_4_Travel_to_P_Start_Exe &&
		t_calib >= T_f_calib) {


		*calib_enc_on = 0;
	}
	///////////////////////////////////////////////////////////////////////////////
	// Calibration timer:
	///////////////////////////////////////////////////////////////////////////////

	if (init_calib_traj)
		step_i = 0;

	///////////////////////////////////////////////////////////////////////////////
	// Generate trajectory points:
	///////////////////////////////////////////////////////////////////////////////

	if (*calib_enc_on)
		traj_linear_points(	p_ref, dt_p_ref, u_t_ref_dum, dt_k,
							p_calib_o, p_calib_f, v_calib, frac_ramp_calib, &init_calib_traj, &T_f_calib,
							&pos_rel_calib_dum, &dt_pos_rel_calib_dum);

	///////////////////////////////////////////////////////////////////////////////
	// ITM console output:
	///////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_OUT_CALIB_CHECK
		if (calib_traj_prev != *calib_traj) { // || step_i % (DT_DISP_MSEC_CALIB/DT_STEP_MSEC) == 0)
			printf("   ----------------------------\n");
			printf("   traj_ref_calibration_ll2(): \n");
			printf("   calib_traj (%d)\t= [%s] \n", *calib_traj, CALIB_TRAJ_STR[*calib_traj]);
			printf("   calib_enc_on\t\t= [%d] \n", *calib_enc_on);
			printf("   t_calib (%d)\t= [%3.2f], T_f_calib = [%3.2f] \n", step_i, t_calib, T_f_calib);

			printf("\n");
			printf("   ax_x = [%3.3f], ax_y = [%3.3f], rot_ang = [%3.3f], traj_exerc_type (%d) = [%s] \n",
					traj_ctrl_params->semiaxis_x, traj_ctrl_params->semiaxis_y, traj_ctrl_params->rot_angle, traj_exerc_type, TRAJ_TYPE_STR[traj_exerc_type]);

			if (*calib_traj == CalibTraj_1_Y_Travel)
				printf("   vel_y = [%3.3f], THR_DT_P_CONTACT = [%3.3f] \n", fabs(dt_p_m[IDX_Y]), THR_DT_P_CONTACT);
			else if (*calib_traj == CalibTraj_2_X_Travel)
				printf("   vel_x = [%3.3f], THR_DT_P_CONTACT = [%3.3f] \n", fabs(dt_p_m[IDX_X]), THR_DT_P_CONTACT);

			printf("   voltages = [%3.3f, %3.3f], THR_VOLTAGE_CONTACT = [%3.3f] \n",
					fabs(LL_motors_settings->left.volt), fabs(LL_motors_settings->right.volt), THR_VOLTAGE_CONTACT);

			if (*calib_traj == CalibTraj_1_Y_Travel || *calib_traj == CalibTraj_2_X_Travel)
				printf("   dist p_calib_o = [%3.3f], ERR_POS_REL = [%3.3f] \n",
					norm2_2d(p_m[IDX_X] - p_calib_o[IDX_X], p_m[IDX_Y] - p_calib_o[IDX_Y]), ERR_POS_REL);

			printf("\n");
			printf("   pos_rel = [%3.3f], dt_pos_rel = [%3.3f]\n", pos_rel_calib_dum, dt_pos_rel_calib_dum);
			printf("   p_ref   = [%3.3f, %3.3f], dt_p_ref = [%3.3f, %3.3f]\n",
				p_ref[IDX_X],
				p_ref[IDX_Y],
				dt_p_ref[IDX_X],
				dt_p_ref[IDX_Y]);

			printf("\n");
			/*
			printf("   p_calib_o = [%3.3f, %3.3f]\n", p_calib_o[IDX_X], p_calib_o[IDX_Y]);
			printf("   p_calib_f = [%3.3f, %3.3f]\n", p_calib_f[IDX_X], p_calib_f[IDX_Y]);
			printf("   D_p_cal   = [%3.3f, %3.3f]\n", p_calib_f[IDX_X] - p_calib_o[IDX_X], p_calib_f[IDX_Y] - p_calib_o[IDX_Y]);
			*/
		}
	#endif

	// Record active calibration trajectory for next iteration:
	calib_traj_prev = *calib_traj;

	// Increase step counter:
	step_i++;
}

///////////////////////////////////////////////////////////////////////////////
// HOMING TRAJECTORY:
///////////////////////////////////////////////////////////////////////////////

void
traj_ref_homing_ll2(double p_ref[], double dt_p_ref[], uint8_t* homing_on, uint8_t* init_home_traj, uint8_t* idx_scale,
	double dt_k, double p_m[], double dt_p_m[], double phi_o, double dt_phi_o,
	traj_ctrl_params_t* traj_ctrl_params, double v_calib, double frac_ramp_calib) {

	const double FACT_T_REF = 1.05; // HAC: extend trajectory reference time to avoid a speed jump during state transition

	// Homing end points:
	static double p_home_o[N_COORD_2D] = {0.0, 0.0};
	static double p_home_f[N_COORD_2D] = {0.0, 0.0};

	// Timers:
	static uint16_t step_i = 0;
	static double T_f_home = 0.0;

	static double t_home   = 0.0;
	t_home = step_i*dt_k;

	// Dummy variables:
	double u_t_ref_dum[N_COORD_2D] = {0.0, 0.0};
	double pos_rel_home_dum        = 0.0;
	double dt_pos_rel_home_dum     = 0.0;

	if (*init_home_traj) { // CRITICAL: this condition differs from what is used in CALIB logic
		// Set up next HOMING trajectory:
		p_home_o[IDX_X] = p_m[IDX_X];
		p_home_o[IDX_Y] = p_m[IDX_Y];

		// HOMING: this will only work with the TRAJ_PARAMS_VARIABLE_OFF option in (LL_sys_info.exercise_state == RUNNING):
		traj_ellipse_points(phi_o, dt_phi_o, p_ref, dt_p_ref, u_t_ref_dum,
				traj_ctrl_params->semiaxis_x, traj_ctrl_params->semiaxis_y, traj_ctrl_params->rot_angle);

		p_home_f[IDX_X] = p_ref[IDX_X];
		p_home_f[IDX_Y] = p_ref[IDX_Y];

		// Control gains scale array:
		idx_scale = IDX_SCALE_CALIB;
	}

	// Generate trajectory points:
	traj_linear_points(	p_ref, dt_p_ref, u_t_ref_dum, dt_k,
						p_home_o, p_home_f, v_calib, frac_ramp_calib, init_home_traj, &T_f_home,
						&pos_rel_home_dum, &dt_pos_rel_home_dum);

	// Increase step counter:
	step_i++;

	// Exit condition:
	if (t_home >= FACT_T_REF*T_f_home)
		*homing_on = 0; // this will cause homing trajectory to stop

	#if USE_ITM_OUT_CALIB_CHECK
		if (step_i % (DT_DISP_MSEC_CALIB/DT_STEP_MSEC) == 0) {
			printf("   ----------------------------\n");
			printf("   traj_ref_homing_ll2(): \n");
			printf("   t_home (%d)\t= [%3.2f], T_f_home = [%3.2f] \n", step_i, t_home, T_f_home);

			printf("\n");
			printf("   pos_rel = [%3.3f], dt_pos_rel = [%3.3f]\n", pos_rel_home_dum, dt_pos_rel_home_dum);
			printf("   p_ref   = [%3.3f, %3.3f], dt_p_ref = [%3.3f, %3.3f]\n",
					p_ref[IDX_X],
					p_ref[IDX_Y],
					dt_p_ref[IDX_X],
					dt_p_ref[IDX_Y]);

			printf("\n");
		}

		if (*homing_on == 0)
			printf("   <<traj_ref_homing_ll2()>> homing_on == 0 \n\n");
	#endif
}
