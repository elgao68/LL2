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
		double phi_home,
		double p_ref[],	double dt_p_ref[],
		double* phi_ref, double* dt_phi_ref,
		double u_t_ref[], double dt_k,
		traj_ctrl_params_t traj_ctrl_params, int8_t mode_traj, int8_t use_traj_params_variable, uint8_t* init_traj) {

	///////////////////////////////////////////////////////////////////////////////
	// COPY TRAJECTORY PARAMETERS TO LOCAL SCOPE VARIABLES (this version makes them static):
	///////////////////////////////////////////////////////////////////////////////

	static double T_cycle;
	static double T_exp;
	static double ax_x;
	static double ax_y;
	static double rot_ang;
	static int cycle_dir;
	static double sig_exp;

	if (*init_traj) {
		T_cycle   = traj_ctrl_params.cycle_period;
		T_exp     = traj_ctrl_params.exp_blend_time;
		ax_x      = traj_ctrl_params.semiaxis_x;
		ax_y      = traj_ctrl_params.semiaxis_y;
		rot_ang   = traj_ctrl_params.rot_angle;
		cycle_dir = (int)traj_ctrl_params.cycle_dir;
		sig_exp   = 3.0/T_exp;
	}

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
	// STATIC VARIABLES declarations (other than trajectory parameters):
	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Cycle frequency - variable fraction:
	///////////////////////////////////////////////////////////////////////////////

	static double frac_dt_phi = 1.0;

	///////////////////////////////////////////////////////////////////////////////
	// Variable trajectory parameters:
	///////////////////////////////////////////////////////////////////////////////

	static double ax_x_adj;
	static double ax_y_adj;

	///////////////////////////////////////////////////////////////////////////////
	// Additional modes & behaviors:
	///////////////////////////////////////////////////////////////////////////////

	static int8_t mode_traj_prev = MODE_TRAJ_NULL; // previous trajectory mode (for switching detection)

	///////////////////////////////////////////////////////////////////////////////
	// Algorithm step counter:
	///////////////////////////////////////////////////////////////////////////////

	static int step_int = 0;

	///////////////////////////////////////////////////////////////////////////////
	// Trajectory initialization - initialize STATIC VARIABLES (CRITICAL):
	///////////////////////////////////////////////////////////////////////////////

	if (*init_traj) {

		// (Re)initialize force fraction:
		frac_dt_phi = 1.0;

		// Variable trajectory parameters:
		if (use_traj_params_variable) {
			ax_x_adj = 0;
			ax_y_adj = 0;
		}
		else {
			ax_x_adj = ax_x;
			ax_y_adj = ax_y;
		}

		// Trajectory mode:
		mode_traj_prev = MODE_TRAJ_NULL;

		// (Re)initialize step counter (CRITICAL):
		step_int = 0;

		*init_traj = 0;
	} // END STATIC VARIABLES declarations

	///////////////////////////////////////////////////////////////////////////////
	// Timers:
	///////////////////////////////////////////////////////////////////////////////

	double t_ref = dt_k*step_int;

	static double t_param_o = 0;

    ///////////////////////////////////////////////////////////////////////////////
    //  Variable trajectory path parameters - behaviors:
    ///////////////////////////////////////////////////////////////////////////////

	if (use_traj_params_variable) {

		static double ax_x_o;
		static double ax_y_o;

		if (mode_traj == MODE_TRAJ_START && mode_traj != mode_traj_prev) {
			t_param_o = t_ref;
			ax_x_o = ax_x_adj;
			ax_y_o = ax_y_adj;
		}
		else if (mode_traj == MODE_TRAJ_END && mode_traj != mode_traj_prev) {
			t_param_o = t_ref;
			ax_x_o = ax_x_adj;
			ax_y_o = ax_y_adj;
		}

		if (mode_traj == MODE_TRAJ_START) {
			ax_x_adj = (1.0 - exp(-sig_exp*(t_ref - t_param_o)))*(ax_x - ax_x_o) + ax_x_o;
			ax_y_adj = (1.0 - exp(-sig_exp*(t_ref - t_param_o)))*(ax_y - ax_y_o) + ax_y_o;
		}
		else if (mode_traj == MODE_TRAJ_END) {
			ax_x_adj = exp(-sig_exp*(t_ref - t_param_o))*ax_x_o;
			ax_y_adj = exp(-sig_exp*(t_ref - t_param_o))*ax_y_o;
		}
	}

    ///////////////////////////////////////////////////////////////////////////////
    //  Fixed trajectory path parameters - behaviors:
    ///////////////////////////////////////////////////////////////////////////////

	else { // (!use_traj_params_variable)
		if (mode_traj == MODE_TRAJ_END && mode_traj != mode_traj_prev)
			t_param_o = t_ref;

		///////////////////////////////////////////////////////////////////////////////
		// Make input force decay:
		///////////////////////////////////////////////////////////////////////////////

		if (mode_traj == MODE_TRAJ_END)
			frac_dt_phi = exp(-sig_exp*(t_ref - t_param_o));
	}

	///////////////////////////////////////////////////////////////////////////////
	// Obtain reference trajectory position and velocity:
	///////////////////////////////////////////////////////////////////////////////

	// Compute instantaneous frequency and phase (NOTE initial condition phi_home):
	*dt_phi_ref = frac_dt_phi*2*PI/T_cycle;
	*phi_ref    = (*dt_phi_ref)*t_ref + phi_home; // TODO: this integral is inexact for the portion where frac_dt_phi decays

	// Compute reference trajectory position and velocity from PHASE:
	traj_ellipse_points(*phi_ref, *dt_phi_ref, p_ref, dt_p_ref, u_t_ref,
		ax_x_adj, ax_y_adj, rot_ang);

    ///////////////////////////////////////////////////////////////////////////////
	// ITM console output:
    ///////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_OUT_TRAJ_REF
		if (step_int == 0) {
			printf("   traj_ref_step_active_elliptic(): \n");
			// printf("   T_cycle = %f\n", T_cycle);
			printf("   T_exp   = %f\n", T_exp);
			printf("   ax_x    = %f\n", ax_x);
			printf("   ax_y    = %f\n", ax_y);
			printf("   rot_ang  = %f\n", rot_ang);
			printf("   cycle_dir = %f\n", (double)cycle_dir);
			printf("\n");
		}

		if (step_int % (DT_DISP_MSEC_ALGO/(int)(1000*dt_k)) == 0) {
			printf("[%d] phi = [%3.2f] \tdt_phi = [%3.2f] \tfrac_dt_phi = [%3.2f] \n\n",
				step_int,
				*phi_ref, *dt_phi_ref,
				frac_dt_phi);
		}
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Store current trajectory mode:
	///////////////////////////////////////////////////////////////////////////////

	mode_traj_prev = mode_traj;

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
	double p_ref[], double dt_p_ref[], uint8_t* calib_enc_on, calib_traj_t* calib_traj, uint8_t* idx_scale_gain,
	double dt_k, double p_m[], double dt_p_m[], double phi_home, double dt_phi_home,
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
			*idx_scale_gain = IDX_SCALE_GAIN_CALIB;

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
			*idx_scale_gain = IDX_SCALE_GAIN_CALIB;

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
			*idx_scale_gain = IDX_SCALE_GAIN_EXERCISE;

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
			*calib_traj = CalibTraj_4_Travel_to_HOME;

			p_calib_o[IDX_X] = 0;
			p_calib_o[IDX_Y] = 0;


			if (traj_exerc_type == EllipticTraj || traj_exerc_type == LinearTraj) {
				// CALIBRATION: this will only work with the TRAJ_PARAMS_VARIABLE_OFF option:
				home_point_ellipse(phi_home, dt_phi_home, p_ref, dt_p_ref, traj_ctrl_params);

				p_calib_f[IDX_X] = p_ref[IDX_X];
				p_calib_f[IDX_Y] = p_ref[IDX_Y];
			}
			else {
				// Safety catch:
				p_calib_f[IDX_X] = 0;
				p_calib_f[IDX_Y] = 0;
			}

			// Control gains scale array:
			*idx_scale_gain = IDX_SCALE_GAIN_EXERCISE;

			// Trajectory initiation command:
			init_calib_traj  = 1;
	}

	// Encoders calibration - exit condition:
	else if (
		*calib_traj == CalibTraj_4_Travel_to_HOME &&
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
		// NOTE: init_calib_traj gets zero'd:
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
traj_ref_homing_ll2(double p_ref[], double dt_p_ref[], uint8_t* homing_traj_on, uint8_t* idx_scale_gain,
	double dt_k, double p_m[], double dt_p_m[], double phi_home, double dt_phi_home,
	traj_ctrl_params_t* traj_ctrl_params, double v_calib, double frac_ramp_calib) {

	const double FACT_T_REF = 1.2; // HACK: extend trajectory reference time to avoid a speed jump during state transition

	// Homing end points:
	static double p_homing_o[N_COORD_2D] = {0.0, 0.0};
	static double p_homing_f[N_COORD_2D] = {0.0, 0.0};

	// Timers:
	static uint16_t step_i = 0;
	double T_f_home;
	double t_home;

	// State variables:
	static uint8_t init_homing_traj = 1; // "initiate homing" flag

	// Dummy variables:
	double u_t_ref_dum[N_COORD_2D] = {0.0, 0.0};
	double pos_rel_home_dum        = 0.0;
	double dt_pos_rel_home_dum     = 0.0;

	///////////////////////////////////////////////////////////////////////////////
	// Initial conditions:
	///////////////////////////////////////////////////////////////////////////////

	if (*homing_traj_on && init_homing_traj) {

		// Set up homing trajectory start & end points:
		p_homing_o[IDX_X] = p_m[IDX_X];
		p_homing_o[IDX_Y] = p_m[IDX_Y];

		home_point_ellipse(phi_home, dt_phi_home, p_ref, dt_p_ref, traj_ctrl_params); // HOMING: this will only work with the TRAJ_PARAMS_VARIABLE_OFF option

		p_homing_f[IDX_X] = p_ref[IDX_X];
		p_homing_f[IDX_Y] = p_ref[IDX_Y];

		// Restart step counter:
		step_i = 0;

		// Control gains scale array:
		*idx_scale_gain = IDX_SCALE_GAIN_EXERCISE; // was IDX_SCALE_GAIN_CALIB
	}

	///////////////////////////////////////////////////////////////////////////////
	// Update timer:
	///////////////////////////////////////////////////////////////////////////////

	t_home = step_i*dt_k;

	///////////////////////////////////////////////////////////////////////////////
	// Generate trajectory points (NOTE: init_homing_traj gets zero'd):
	///////////////////////////////////////////////////////////////////////////////

	traj_linear_points(	p_ref, dt_p_ref, u_t_ref_dum, dt_k,
						p_homing_o, p_homing_f, v_calib, frac_ramp_calib, &init_homing_traj, &T_f_home,
						&pos_rel_home_dum, &dt_pos_rel_home_dum);

	///////////////////////////////////////////////////////////////////////////////
	// Exit condition:
	///////////////////////////////////////////////////////////////////////////////

	if (t_home >= FACT_T_REF*T_f_home) {
		*homing_traj_on  = 0; // this will cause homing trajectory to stop
		init_homing_traj = 1; // re-arm "initiate homing" flag for next homing call
	}

	///////////////////////////////////////////////////////////////////////////////
	// ITM Console output:
	///////////////////////////////////////////////////////////////////////////////

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

		if (*homing_traj_on == 0)
			printf("   <<traj_ref_homing_ll2()>> homing_traj_on = [%d] \n\n", *homing_traj_on);
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Increase step counter:
	///////////////////////////////////////////////////////////////////////////////

	step_i++;
}

void
home_point_ellipse(double phi_home, double dt_phi_home, double p_ref[], double dt_p_ref[], traj_ctrl_params_t* traj_ctrl_params) {
	// Dummy variables:
	double u_t_ref_dum[N_COORD_2D] = {0.0, 0.0};

	traj_ellipse_points(phi_home, dt_phi_home, p_ref, dt_p_ref, u_t_ref_dum,
		traj_ctrl_params->semiaxis_x, traj_ctrl_params->semiaxis_y, traj_ctrl_params->rot_angle);
}
