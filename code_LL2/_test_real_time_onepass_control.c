/////////////////////////////////////////////////////////////////////////////
//
//  _test_real_time_onepass_control.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <_test_real_time.h>

extern lowerlimb_sys_info_t lowerlimb_sys_info;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// TEST SCRIPT - REAL-TIME
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_real_time_onepass_control(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3) {

	///////////////////////////////////////////////////////////////////////////////
	// General settings:
	///////////////////////////////////////////////////////////////////////////////

	const uint8_t USE_SOFTWARE_MSG_LIST = 0; // CRITICAL option

	///////////////////////////////////////////////////////////////////////////////
	// MOTOR STATE VARS:
	///////////////////////////////////////////////////////////////////////////////

	uint8_t motor_alert         = 0;
	uint8_t motor_torque_active = 0;

	///////////////////////////////////////////////////////////////////////////////
	// FIRMWARE / CONTROL PARAMETERS:
	///////////////////////////////////////////////////////////////////////////////

	uint64_t up_time;
	uint64_t up_time_end;

	lowerlimb_mech_readings_t   LL_mech_readings;
	lowerlimb_motors_settings_t LL_motors_settings;
	traj_ctrl_params_t          traj_ctrl_params;
	admitt_model_params_t       admitt_model_params;
	lowerlimb_ref_kinematics_t	ref_kinematics;

	const int is_calibration = 1; // what did this flag do in update_motor_algo() (now set_LL_mech_readings())?

	//////////////////////////////////////////////////////////////////////////////////
	// ADMITTANCE & TRAJECTORY PARAMETERS:
	//////////////////////////////////////////////////////////////////////////////////

	#if OVERR_DYN_PARAMS_RT

		//////////////////////////////////////////////////////////////////////////////////
		// Trajectory parameters:
		//////////////////////////////////////////////////////////////////////////////////

		traj_ctrl_params.cycle_period   = CYCLE_PERIOD_DEF;
		traj_ctrl_params.exp_blend_time = EXP_BLEND_TIME;
		traj_ctrl_params.semiaxis_x     = SEMIAXIS_X_DEF;
		traj_ctrl_params.semiaxis_y     = SEMIAXIS_Y_DEF;
		traj_ctrl_params.rot_angle      = ROT_ANGLE_DEF;
		traj_ctrl_params.cycle_dir      = CYCLE_DIR_DEF;

		//////////////////////////////////////////////////////////////////////////////////
		// Dynamic response parameters:
		//////////////////////////////////////////////////////////////////////////////////

		double damp_ratio = DAMP_RATIO_DEF;
		double w_n        = OMEGA_N_DEF; // natural frequency
		double sigma      = damp_ratio*w_n;

		double T_exp      = traj_ctrl_params.exp_blend_time;

		//////////////////////////////////////////////////////////////////////////////////
		// Admittance parameters:
		//////////////////////////////////////////////////////////////////////////////////

		admitt_model_params.inertia_x   = INERTIA_XY_DEF;
		admitt_model_params.inertia_y   = INERTIA_XY_DEF;
		admitt_model_params.inertia_phi = INERTIA_PHI_DEF;

		if (ADMITT_MODEL_CONSTR_ON) {
			admitt_model_params.stiffness = STIFFNESS_DEF;
			admitt_model_params.damping   = DAMPING_DEF;
		}
		else {
			admitt_model_params.stiffness = admitt_model_params.inertia_x*w_n*w_n;
			admitt_model_params.damping   = 2*damp_ratio*sqrt(admitt_model_params.stiffness*admitt_model_params.inertia_x);
		}

		admitt_model_params.p_eq_x    = 0;
		admitt_model_params.p_eq_y    = 0;
		admitt_model_params.Fx_offset = 0;
		admitt_model_params.Fy_offset = 0;

		admitt_model_params.F_tang_magn = F_TANG_DEF;
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Select trajectory type:
	///////////////////////////////////////////////////////////////////////////////

	uint8_t traj_exerc_type;

	if (traj_ctrl_params.semiaxis_x == 0 &&	traj_ctrl_params.semiaxis_y == 0)
		 traj_exerc_type = IsometricTraj;
	else if (traj_ctrl_params.semiaxis_x != 0 && traj_ctrl_params.semiaxis_y == 0)
		 traj_exerc_type = LinearTraj;
	else
		 traj_exerc_type = EllipticTraj;

	#if USE_ITM_OUT_RT_CHECK
		printf("   test_real_time: traj_exerc_type = [%s]\n\n", TRAJ_TYPE_STR[traj_exerc_type]);
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Real-time time step, sec:
	///////////////////////////////////////////////////////////////////////////////

	double dt_k = (double)DT_STEP_MSEC/MSEC_PER_SEC;

	///////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - REFERENCE:
	///////////////////////////////////////////////////////////////////////////////

	// Reference position and velocity:
	double    p_ref[N_COORD_2D] = {0.0, 0.0};
	double dt_p_ref[N_COORD_2D] = {0.0, 0.0};

	// Cycle phase and instantaneous frequency:
	double    phi_ref = 0.0;
	double dt_phi_ref = 0.0;

	// Trajectory path tangent vector:
	double u_t_ref[N_COORD_2D] = {0.0, 0.0};

	// Internal state: initial values:
	double z_intern_o_dbl[2*N_COORD_EXT];

	double    phi_o = PHI_INIT_EXERC;
	double dt_phi_o = 0.0;

	///////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - MEASURED:
	///////////////////////////////////////////////////////////////////////////////

	// Measured position and velocity:
	double    p_m[N_COORD_2D] = {0.0, 0.0};
	double dt_p_m[N_COORD_2D] = {0.0, 0.0};

	///////////////////////////////////////////////////////////////////////////////
	// Force sensor variables:
	///////////////////////////////////////////////////////////////////////////////

	// Force sensor readings, raw:
	uint32_t force_end_in_x_sensor = 0;
	uint32_t force_end_in_y_sensor = 0;

	// End-effector force measurements:
	double F_end_m[N_COORD_2D]  = {0.0, 0.0};
	double F_end_in[N_COORD_2D] = {0.0, 0.0}; // for admittance / active trajectory control - may differ from F_end_m

	///////////////////////////////////////////////////////////////////////////////
	// Force sensor variables - dynamic gravity compensation:
	///////////////////////////////////////////////////////////////////////////////

	double OMEGA_LO_F_END            = 2*PI*0.3;

	double F_end_lo[N_COORD_2D]      = {0.0, 0.0};
	double F_end_lo_prev[N_COORD_2D] = {0.0, 0.0};

	double F_GCOMP_LIM = 45.0;

	double sig_F_gcomp = 3.0/F_GCOMP_LIM;
	double F_gcomp_dyn;

	///////////////////////////////////////////////////////////////////////////////
	// Gain and scaling variables:
	///////////////////////////////////////////////////////////////////////////////

	uint8_t idx_scale = IDX_SCALE_EXERCISE; // scale arrays selector, activity-based (CRITICAL)

	///////////////////////////////////////////////////////////////////////////////
	// Force commands:
	///////////////////////////////////////////////////////////////////////////////

	float F_end_cmd_fb[N_COORD_2D]      = {0.0, 0.0}; // FB force command
	float F_end_cmd_ff[N_COORD_2D]      = {0.0, 0.0}; // FF force command
	float F_end_cmd_gcomp[N_COORD_2D]   = {0.0, 0.0}; // gravity compensation force command
	float F_end_cmd_int_err[N_COORD_2D] = {0.0, 0.0}; // integral position error force command

	float F_end_cmd[N_COORD_2D]         = {0.0, 0.0}; // total force command

	///////////////////////////////////////////////////////////////////////////////
	// Additional control variables:
	///////////////////////////////////////////////////////////////////////////////

	// Position & velocity error vector:
	double err_pos_vel[N_POS_VEL_2D]    = {0.0, 0.0, 0.0, 0.0};

	// Integral position error:
	double OMEGA_HI_ERR_INT_POS         = 2*PI*0.2; // hi-pass filter cutoff frequency

	double err_pos[N_COORD_2D]          = {0.0, 0.0};
	double err_int_pos[N_COORD_2D]      = {0.0, 0.0};
	double err_int_pos_prev[N_COORD_2D] = {0.0, 0.0};

	// Exercise substate - SLOWING:
	double t_slow   = 0.0; // TODO: consider using an implementation that doesn't require an explicit time reference (local step counts?)
	double t_slow_ref = 0.0;

	///////////////////////////////////////////////////////////////////////////////
	// CALIBRATION variables:
	///////////////////////////////////////////////////////////////////////////////

	calib_traj_t calib_traj = CalibTraj_Null;
	uint8_t calib_enc_on    = 0;

	///////////////////////////////////////////////////////////////////////////////
	// HOMING variables:
	///////////////////////////////////////////////////////////////////////////////

	double OMEGA_THR_HOMING_START = 0.03;

	uint8_t homing_on      = 0; // CRITICAL initialization
	// uint8_t init_homing_traj = 1; // TODO: remove at a later date

	///////////////////////////////////////////////////////////////////////////////
	// IDLE activity state variables:
	///////////////////////////////////////////////////////////////////////////////

	uint8_t init_idle_activity_state = 1;

	///////////////////////////////////////////////////////////////////////////////
	// TCP/IP variables:
	///////////////////////////////////////////////////////////////////////////////

	int sock_status;

	// TCP communication checks:
	int32_t ret_tcp_msg      = 0;
	uint64_t dt_ret_tcp_msec = 0;

	///////////////////////////////////////////////////////////////////////////////
	// State variables:
	///////////////////////////////////////////////////////////////////////////////

	uint16_t cmd_code      = NO_CMD;
	uint16_t cmd_code_prev = NO_CMD;
	uint8_t  app_state;

	uint8_t exercise_state_prev = lowerlimb_sys_info.exercise_state;

	///////////////////////////////////////////////////////////////////////////////
	// Real-time counters, timers and switches:
	///////////////////////////////////////////////////////////////////////////////

	double T_RUN_MAX   = 5000;
	double t_ref       = 0.0;

	int rt_step_i      = 0; // real-time step counter
	int r_i, c_i; // general-purpose counters

	int8_t switch_traj = SWITCH_TRAJ_NULL;

	///////////////////////////////////////////////////////////////////////////////
	// Display variables:
	///////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_OUT_RT_CHECK
		uint8_t idx_sys_state;
		uint8_t idx_activ_state;
		uint8_t idx_exerc_state;
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Initialize app:
	///////////////////////////////////////////////////////////////////////////////

	app_state = lowerlimb_app_state_initialize(0, VER_H, VER_L, VER_P, &LL_motors_settings);

	///////////////////////////////////////////////////////////////////////////////
	// Initialize motor algorithm:
	///////////////////////////////////////////////////////////////////////////////

	init_motor_algo(&LL_mech_readings, &LL_motors_settings);

	///////////////////////////////////////////////////////////////////////////////
	// Control settings:
	///////////////////////////////////////////////////////////////////////////////

	nml_mat* K_lq_xv_nml;
	nml_mat* F_end_cmd_fb_nml;
	nml_mat* err_pos_vel_nml;

	static int init_nml = 1;

	if (init_nml) {
		K_lq_xv_nml      = nml_mat_from(N_COORD_2D, N_POS_VEL_2D, N_COORD_2D*N_POS_VEL_2D, K_LQ_XV_DEF);
		F_end_cmd_fb_nml = nml_mat_new(N_COORD_2D,   1);
		err_pos_vel_nml  = nml_mat_new(N_POS_VEL_2D, 1);

		init_nml = 0;
	}

	#if USE_ITM_OUT_RT_CHECK
		printf("test_real_time_onepass_control():\n\n");
		printf("K_lq_xv_nml:\n");
		for (r_i = 0; r_i < K_lq_xv_nml->num_rows; r_i++) {
			for (c_i = 0; c_i < K_lq_xv_nml->num_cols; c_i++)
				printf("%3.4f\t", K_lq_xv_nml->data[r_i][c_i]);
			printf("\n");
		}
		printf("\n");
	#endif

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
	// USER CODE LOOP:
	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	while (t_ref <= T_RUN_MAX) {

		///////////////////////////////////////////////////////////////////////////////
		// Get uptime in milliseconds:
		///////////////////////////////////////////////////////////////////////////////

		up_time = getUpTime();

		///////////////////////////////////////////////////////////////////////////////
		// Execute real-time step:
		///////////////////////////////////////////////////////////////////////////////

		if (up_time >= algo_nextTime) { // RT time step

			///////////////////////////////////////////////////////////////////////////////
			// Set up next RT time point (msec):
			///////////////////////////////////////////////////////////////////////////////

			algo_nextTime = up_time + DT_STEP_MSEC;

			///////////////////////////////////////////////////////////////////////////////
			// Update reference time:
			///////////////////////////////////////////////////////////////////////////////

			t_ref = dt_k*rt_step_i;

			///////////////////////////////////////////////////////////////////////////////
			// Ethernet connection:
			///////////////////////////////////////////////////////////////////////////////

			#if USE_ITM_TCP_CHECK
				dt_ret_tcp_msec = getUpTime(); //  tests time elapsed waiting for ethernet_w5500_state() to return
			#endif

			ret_tcp_msg = ethernet_w5500_state(&sock_status);

			#if USE_ITM_TCP_CHECK
				dt_ret_tcp_msec = getUpTime() - dt_ret_tcp_msec;

				if (sock_status != SOCK_ESTABLISHED) { // (ret_tcp_msg < 0)
					// Current socket state:
					printf("ethernet_w5500_state(): SOCKET STATUS [0x%02x]\n", sock_status);

					// Current TCP return message and time:
					printf("rt_step_i [%d]: cmd_code = [%d], ret_tcp_msg = [%d], dt_ret_tcp_msec = [%d]\n\n", rt_step_i, cmd_code, ret_tcp_msg, (int)dt_ret_tcp_msec);
				}
			#endif

			//////////////////////////////////////////////////////////////////////////////////
			// GET TCP/IP APP STATE
			//////////////////////////////////////////////////////////////////////////////////

			// CRITICAL: passes reference to global-declared lowerlimb_sys_info for the sake of traceability:
			lowerlimb_app_onepass_ref(&lowerlimb_sys_info, Read_Haptic_Button(), motor_alert,
				&traj_ctrl_params, &admitt_model_params, &LL_motors_settings, &cmd_code,
				&calib_enc_on, USE_SOFTWARE_MSG_LIST);

			// HACK: exercise state overrides:
			if (lowerlimb_sys_info.exercise_state == SETUP)
				lowerlimb_sys_info.exercise_state = RUNNING;

			///////////////////////////////////////////////////////////////////////////////
			// Clear motor_alert after sending it to TCP/IP APP state:
			///////////////////////////////////////////////////////////////////////////////

			motor_alert = 0;

			#if USE_ITM_OUT_GUI_PARAMS
				if (rt_step_i > 0 && rt_step_i % (DT_DISP_MSEC_GUI_PARAMS/DT_STEP_MSEC) == 0) {
					printf("___________________________________\n");
					printf("rt_step_i   = [%d]\n", rt_step_i);
					printf("\n");
					printf("cycle_period   = [%f]\n", traj_ctrl_params.cycle_period);
					printf("exp_blend_time = [%f]\n", traj_ctrl_params.exp_blend_time );
					printf("semiaxis_x     = [%f]\n", traj_ctrl_params.semiaxis_x);
					printf("semiaxis_y     = [%f]\n", traj_ctrl_params.semiaxis_y);
					printf("rot_angle      = [%f]\n", traj_ctrl_params.rot_angle);
					// printf("cycle_dir    = [%d]\n", traj_ctrl_params.cycle_dir);

					printf("\n");
					printf("inertia_x = [%f]\n", admitt_model_params.inertia_x );
					printf("inertia_y = [%f]\n", admitt_model_params.inertia_y);
					printf("damping   = [%f]\n", admitt_model_params.damping);
					printf("stiffness = [%f]\n", admitt_model_params.stiffness);
					printf("p_eq_x    = [%f]\n", admitt_model_params.p_eq_x );
					printf("p_eq_y    = [%f]\n", admitt_model_params.p_eq_y);
					printf("Fx_offset = [%f]\n", admitt_model_params.Fx_offset);
					printf("Fy_offset = [%f]\n", admitt_model_params.Fy_offset);
					printf("\n");

					fflush(stdout);
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// SYSTEM STATE SWITCH:
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			if (lowerlimb_sys_info.system_state == SYS_ON) {

				///////////////////////////////////////////////////////////////////////////////
				// Update LEDs state:
				///////////////////////////////////////////////////////////////////////////////

				cycle_haptic_buttons();

				///////////////////////////////////////////////////////////////////////////////
				// Update safety:
				///////////////////////////////////////////////////////////////////////////////

				set_safetyOff(lowerlimb_sys_info.safetyOFF);

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// SENSOR readings:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				// Retrieve force sensor readings:
				///////////////////////////////////////////////////////////////////////////////

				force_sensors_read(hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor, &dum_force_end_in_x, &dum_force_end_in_y);
				current_sensors_read(hadc1, &current_sensor_L, &current_sensor_R);

				///////////////////////////////////////////////////////////////////////////////
				// Get lower-limb robot sensor readings:
				///////////////////////////////////////////////////////////////////////////////

				motor_alert = set_LL_mech_readings(&LL_mech_readings, up_time,
					qei_count_L_read(), qei_count_R_read(),
					current_sensor_L, current_sensor_R,
					force_end_in_x_sensor, force_end_in_y_sensor,
					is_calibration);

				#if OVERR_FORCE_SENSORS_CALIB
					LL_mech_readings.Xforce = 0;
					LL_mech_readings.Yforce = 0;
				#endif

				///////////////////////////////////////////////////////////////////////////////
				// Extract measured position and velocity from sensor readings:
				///////////////////////////////////////////////////////////////////////////////

				p_m[IDX_X]      = (double)LL_mech_readings.coord.x;
				p_m[IDX_Y]      = (double)LL_mech_readings.coord.y;

				dt_p_m[IDX_X]   = (double)LL_mech_readings.velocity.x;
				dt_p_m[IDX_Y]   = (double)LL_mech_readings.velocity.y;

				///////////////////////////////////////////////////////////////////////////////
				// Extract measured end-effector forces:
				///////////////////////////////////////////////////////////////////////////////

				F_end_m[IDX_X]  = (double)LL_mech_readings.Xforce;
				F_end_m[IDX_Y]  = (double)LL_mech_readings.Yforce;

				F_end_in[IDX_X] = SCALE_F_END_MEAS*F_end_m[IDX_X];
				F_end_in[IDX_Y] = SCALE_F_END_MEAS*F_end_m[IDX_Y];

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// EXERCISE start / stop switches:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				if (exercise_state_prev != RUNNING && lowerlimb_sys_info.exercise_state == RUNNING)
					switch_traj = SWITCH_TRAJ_START;
				else if (exercise_state_prev != SLOWING && lowerlimb_sys_info.exercise_state == SLOWING)
					switch_traj = SWITCH_TRAJ_END;
				else
					switch_traj = SWITCH_TRAJ_NULL;

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// ACTIVITY STATE SWITCH:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				// Scale array selector (CRITICAL):
				///////////////////////////////////////////////////////////////////////////////

				if (lowerlimb_sys_info.activity_state != CALIB)
					idx_scale = IDX_SCALE_EXERCISE;

				///////////////////////////////////////////////////////////////////////////////
				// IDLE activity state:
				///////////////////////////////////////////////////////////////////////////////

				if (lowerlimb_sys_info.activity_state == IDLE) {

					// Default kinematic reference:
					if (init_idle_activity_state) {
						p_ref[IDX_X]    = p_m[IDX_X];
						p_ref[IDX_Y]    = p_m[IDX_Y];

						dt_p_ref[IDX_X] = 0.0;
						dt_p_ref[IDX_Y] = 0.0;

						init_idle_activity_state = 0;

						#if USE_ITM_OUT_RT_CHECK
							idx_sys_state   = lowerlimb_sys_info.system_state;
							idx_activ_state = lowerlimb_sys_info.activity_state;
							idx_exerc_state = lowerlimb_sys_info.exercise_state;

							printf("   <<test_real_time_onepass_control()>> [IDLE]:\n");
							printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
							printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
							printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
							printf("\n");
						#endif
					}
				}

				///////////////////////////////////////////////////////////////////////////////
				// CALIBRATION activity state:
				///////////////////////////////////////////////////////////////////////////////

				else if (lowerlimb_sys_info.activity_state == CALIB) { // NOTE: calib_enc_on condition is activated by lowerlimb_app_onepass_ref()

					#if USE_ITM_OUT_RT_CHECK
						if (cmd_code != cmd_code_prev) {
							printf("   <<test_real_time_onepass_control()>> cmd_code_prev = [%s], cmd_code = [%s]\n\n", CMD_STR[cmd_code_prev], CMD_STR[cmd_code]);
						}
					#endif

					// NOTE: calib_enc_on will produce activity state transition from CALIB in lowerlimb_app_onepass_ref():
					traj_ref_calibration_ll2(
						p_ref, dt_p_ref, &calib_enc_on, &calib_traj, &idx_scale, z_intern_o_dbl,
						dt_k, p_m, dt_p_m, phi_o, dt_phi_o,
						&LL_motors_settings, &traj_ctrl_params, traj_exerc_type, V_CALIB, FRAC_RAMP_CALIB);

				} // end if (lowerlimb_sys_info.activity_state == CALIB)

				///////////////////////////////////////////////////////////////////////////////
				// JOG activity state:
				///////////////////////////////////////////////////////////////////////////////

				else if (lowerlimb_sys_info.activity_state == JOG) {

				} // end if (lowerlimb_sys_info.activity_state == JOG)

				///////////////////////////////////////////////////////////////////////////////
				// EXERCISE activity state:
				///////////////////////////////////////////////////////////////////////////////

				else if (lowerlimb_sys_info.activity_state == EXERCISE) {

					///////////////////////////////////////////////////////////////////////////////
					// Exercise substate switch:
					///////////////////////////////////////////////////////////////////////////////

					if (lowerlimb_sys_info.exercise_state == RUNNING || lowerlimb_sys_info.exercise_state == SLOWING) {

						///////////////////////////////////////////////////////////////////////////////
						// SLOWING substate: reference time
						///////////////////////////////////////////////////////////////////////////////

						if (lowerlimb_sys_info.exercise_state == SLOWING && exercise_state_prev != SLOWING) {
							t_slow_ref = t_ref;

							#if USE_ITM_OUT_RT_CHECK
								idx_sys_state   = lowerlimb_sys_info.system_state;
								idx_activ_state = lowerlimb_sys_info.activity_state;
								idx_exerc_state = lowerlimb_sys_info.exercise_state;

								printf("   <<test_real_time_onepass_control()>> [SLOWING] (no cmd code):\n");
								printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
								printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
								printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
								printf("\n");
								printf("   t_slow_ref = [%3.2f]\n", t_slow_ref);
								printf("\n");
							#endif
						}

						///////////////////////////////////////////////////////////////////////////////
						// Generate reference trajectory:
						///////////////////////////////////////////////////////////////////////////////

						// Passive trajectory control:
						if (lowerlimb_sys_info.exercise_mode == PassiveTrajectoryCtrl) {
							if (traj_exerc_type == EllipticTraj || traj_exerc_type == LinearTraj) {
								traj_ref_step_passive_elliptic(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref, dt_k,
									traj_ctrl_params, switch_traj, TRAJ_PARAMS_VARIABLE_OFF); // NOTE: was TRAJ_PARAMS_VARIABLE_ON for both cases
							}
							else
								// Isometric trajectory OR safety catch:
								traj_ref_step_isometric(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref);
						}

						// Active trajectory control:
						else if (lowerlimb_sys_info.exercise_mode == ActiveTrajectoryCtrl) {
							if (traj_exerc_type == EllipticTraj || traj_exerc_type == LinearTraj)
								traj_ref_step_active_elliptic(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref, dt_k, F_end_in, z_intern_o_dbl,
									traj_ctrl_params, admitt_model_params, ADMITT_MODEL_CONSTR_ON, switch_traj, TRAJ_PARAMS_VARIABLE_OFF); // NOTE: elliptic was TRAJ_PARAMS_VARIABLE_ON
							else
								// Isometric trajectory OR safety catch:
								traj_ref_step_isometric(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref);
						}

						///////////////////////////////////////////////////////////////////////////////
						// SLOWING exercise substate: detect "ready for HOMING" activity state
						///////////////////////////////////////////////////////////////////////////////

						if (lowerlimb_sys_info.exercise_state == SLOWING) {
							t_slow = t_ref - t_slow_ref;

							if (t_slow > T_exp && fabs(dt_phi_ref) < OMEGA_THR_HOMING_START) {
								homing_on      = 1; // HACK: specifically to bypass [lowerlimb_sys_info.exercise_state] returned by [lowerlimb_app_onepass_ref()]
								// init_homing_traj = 1; // TODO: remove at a later date
								lowerlimb_sys_info.activity_state = HOMING;
								lowerlimb_sys_info.exercise_state = STOPPED;

								#if USE_ITM_OUT_RT_CHECK
									printf("   <<test_real_time_onepass_control()>> HOMING condition detected \n\n");
									// printf("   t_slow = [%3.2f], t_slow_ref = [%3.2f]\n\n", t_slow, t_slow_ref);
								#endif
							}
						}
					} // end if (lowerlimb_sys_info.exercise_state == RUNNING || lowerlimb_sys_info.exercise_state == SLOWING)

					///////////////////////////////////////////////////////////////////////////////
					// Invalid exercise substate:
					///////////////////////////////////////////////////////////////////////////////

					else {
						#if USE_ITM_OUT_RT_CHECK
							printf("   <<test_real_time_onepass_control()>> Invalid exercise substate [%s] for activity_state == EXERCISE] \n\n", EXERC_STATE_STR[lowerlimb_sys_info.exercise_state]);
						#endif
					}
				} // end if (lowerlimb_sys_info.activity_state == EXERCISE)

				///////////////////////////////////////////////////////////////////////////////
				// HOMING activity state:
				///////////////////////////////////////////////////////////////////////////////

				else if (lowerlimb_sys_info.activity_state == HOMING) {

					/*
					#if USE_ITM_OUT_RT_CHECK
						if (init_homing_traj) {
							idx_sys_state   = lowerlimb_sys_info.system_state;
							idx_activ_state = lowerlimb_sys_info.activity_state;
							idx_exerc_state = lowerlimb_sys_info.exercise_state;

							printf("   <<test_real_time_onepass_control()>> [HOMING] (no cmd code): \n");
							printf("   system_state:   [%s]\n", SYS_STATE_STR[idx_sys_state]  );
							printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
							printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
							printf("\n");
						}
					#endif
					*/

					traj_ref_homing_ll2(p_ref, dt_p_ref, &homing_on, &idx_scale, // was &homing_on, &init_homing_traj, &idx_scale
						dt_k, p_m, dt_p_m, phi_o, dt_phi_o,
						&traj_ctrl_params, V_CALIB, FRAC_RAMP_CALIB);

					// Exit condition:
					if (!homing_on) {
						lowerlimb_sys_info.activity_state = IDLE;
						init_idle_activity_state = 1;
					}
				} // end if (lowerlimb_sys_info.activity_state == EXERCISE)

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// Set reference kinematics struct:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				for (int c_i = 0; c_i < N_COORD_2D; c_i++) {
					ref_kinematics.p_ref[c_i]    = p_ref[c_i];
					ref_kinematics.dt_p_ref[c_i] = dt_p_ref[c_i];
				}

				ref_kinematics.phi_ref    = phi_ref;
				ref_kinematics.dt_phi_ref = dt_phi_ref;

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// End-effector force commands:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				// Gravity compensation force command (end-effector coordinates):
				///////////////////////////////////////////////////////////////////////////////

				// Low-pass filtered sensor forces:
				for (c_i = IDX_X; c_i <= IDX_Y; c_i++) {
					F_end_lo[c_i] = low_pass_discrete_fwd(F_end_m[c_i], F_end_lo_prev[c_i],
						OMEGA_LO_F_END, (double)DT_STEP_MSEC/MSEC_PER_SEC);

					F_end_lo_prev[c_i] = F_end_lo[c_i];
				}

				// Dynamic gravity compensation:
				F_gcomp_dyn = F_GCOMP_LIM*(1 - exp(-sig_F_gcomp*fmax(-F_end_lo[IDX_Y], 0)));

				// Total gravity compensation:
				F_end_cmd_gcomp[IDX_X] = 0;
				F_end_cmd_gcomp[IDX_Y] = SCALE_GCOMP[idx_scale]*F_G_COMP_DEF + F_gcomp_dyn;

				///////////////////////////////////////////////////////////////////////////////
				// FB force command:
				///////////////////////////////////////////////////////////////////////////////

				// Kinematics error vector, array form:
				err_pos_vel[0] =    p_ref[IDX_X] -    p_m[IDX_X];
				err_pos_vel[1] = dt_p_ref[IDX_X] - dt_p_m[IDX_X];

				err_pos_vel[2] =    p_ref[IDX_Y] -    p_m[IDX_Y];
				err_pos_vel[3] = dt_p_ref[IDX_Y] - dt_p_m[IDX_Y];

				// Kinematics error vector, nml matrix form:
				for (c_i = 0; c_i < N_POS_VEL_2D; c_i++)
					err_pos_vel_nml->data[c_i][0] = err_pos_vel[c_i];

				// FB force command, nml matrix form:
				nml_mat_dot_ref(F_end_cmd_fb_nml, K_lq_xv_nml, err_pos_vel_nml);
				nml_mat_smult_r(F_end_cmd_fb_nml, SCALE_FB[idx_scale]); // scale by switching variable

				// FB force commands, array form:
				for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
					F_end_cmd_fb[c_i] = F_end_cmd_fb_nml->data[c_i][0];

				///////////////////////////////////////////////////////////////////////////////
				// FF force command:
				///////////////////////////////////////////////////////////////////////////////

				// FF force commands, array form:
				for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
					F_end_cmd_ff[c_i] = SCALE_FF[idx_scale]*C_FF_DC_DEF[c_i]*dt_p_ref[c_i];

				///////////////////////////////////////////////////////////////////////////////
				// Integral position error force command:
				///////////////////////////////////////////////////////////////////////////////

				// Integral position error:
				err_pos[IDX_X] = err_pos_vel[0];
				err_pos[IDX_Y] = err_pos_vel[2];

				for (c_i = IDX_X; c_i <= IDX_Y; c_i++) {
					err_int_pos[c_i] = int_hi_pass_discrete_fwd(err_pos[c_i], err_int_pos_prev[c_i],
						OMEGA_HI_ERR_INT_POS, (double)DT_STEP_MSEC/MSEC_PER_SEC);

					err_int_pos_prev[c_i] = err_int_pos[c_i];
				}

				// Force command:
				for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
					F_end_cmd_int_err[c_i] = SCALE_INT_ERR[idx_scale]*err_int_pos[c_i];

				///////////////////////////////////////////////////////////////////////////////
				// Total force command:
				///////////////////////////////////////////////////////////////////////////////

				for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
					F_end_cmd[c_i] = F_end_cmd_fb[c_i] + F_end_cmd_ff[c_i] + F_end_cmd_gcomp[c_i] + F_end_cmd_int_err[c_i];

				///////////////////////////////////////////////////////////////////////////////
				// Update motors' settings:
				///////////////////////////////////////////////////////////////////////////////

				clear_lowerlimb_motors_settings(&LL_motors_settings);
				set_LL_motor_settings(&LL_motors_settings, F_end_cmd);

				///////////////////////////////////////////////////////////////////////////////
				// Send motor commands:
				///////////////////////////////////////////////////////////////////////////////

				// Check motor activation status:
				if (      lowerlimb_sys_info.activity_state == CALIB    && MOTOR_TORQUE_ACTIVE_CALIB)
								motor_torque_active = 1; // NOTE: this "clamps" motor activation even in future IDLE states
				else if ((lowerlimb_sys_info.activity_state == JOG      && MOTOR_TORQUE_ACTIVE_JOG == 0     ) ||
						 (lowerlimb_sys_info.activity_state == EXERCISE && MOTOR_TORQUE_ACTIVE_EXERCISE == 0) )
								motor_torque_active = 0;

				// Issue motion commands to motors - CRITICAL:
				if (motor_alert != 1 && motor_alert != 2) {
					if (motor_torque_active) {
						motor_L_move(LL_motors_settings.left.dac_in,  LL_motors_settings.left.motor_direction,  LL_motors_settings.left.en_motor_driver);
						motor_R_move(LL_motors_settings.right.dac_in, LL_motors_settings.right.motor_direction, LL_motors_settings.right.en_motor_driver);
					}
				}
				else {
					stop_exercise(&LL_motors_settings);

					// Disable motors:
					motor_L_move(0, false, false);
					motor_R_move(0, false, false);

					// Display section:
					#if USE_ITM_OUT_RT_CHECK
						printf("\n");
						printf("   <<test_real_time_onepass_control()>> MOTOR ALERT = [%d] \n\n", motor_alert);
					#endif
				}

				///////////////////////////////////////////////////////////////////////////////
				// Send feedback data to TCP/IP app:
				///////////////////////////////////////////////////////////////////////////////

				// HACK: include low-pass filtered force sensor measurements in robot readings:
				LL_mech_readings.Xforce = F_end_lo[IDX_X];
				LL_mech_readings.Yforce = F_end_lo[IDX_Y];

				send_lowerlimb_exercise_feedback(up_time, &LL_mech_readings, &LL_motors_settings, &ref_kinematics);

				///////////////////////////////////////////////////////////////////////////////
				// ITM console output:
				///////////////////////////////////////////////////////////////////////////////

				/*
				#if USE_ITM_OUT_RT_CHECK
					if ((up_time_end - up_time) > DT_STEP_MSEC) {
						printf("____________________________\n");
						printf("rt_step_i [%d]: t_ref = %f\tDT MSEC = %d\n",
							rt_step_i,
							t_ref,
							(int)up_time_end - (int)up_time;
						printf("____________________________\n");
					}
				#endif
				*/

			} // end if (lowerlimb_sys_info.system_state == ON)

			else { // lowerlimb_sys_info.system_state != ON
				// Change LEDs to system state OFF:
				LED_sys_state_off();

				// Disable motors:
				motor_L_move(0, false, false);
				motor_R_move(0, false, false);

				// Engage brakes:
				l_brakes(ENGAGE_BRAKES);
				r_brakes(ENGAGE_BRAKES);

				// Safety catch: reset integral error
				err_int_pos[IDX_X] = 0;
				err_int_pos[IDX_Y] = 0;
			}

			///////////////////////////////////////////////////////////////////////////////
			// Track changes of state:
			///////////////////////////////////////////////////////////////////////////////

			exercise_state_prev = lowerlimb_sys_info.exercise_state;

			cmd_code_prev = cmd_code;

			///////////////////////////////////////////////////////////////////////////////
			// ITM console output:
			///////////////////////////////////////////////////////////////////////////////

			#if USE_ITM_OUT_RT_CHECK_LONG
				if (rt_step_i % (DT_DISP_MSEC_REALTIME/DT_STEP_MSEC) == 0) {
					// Check uptime after computations:
					up_time_end = getUpTime();

					printf("   ----------------------------\n");
					printf("   %d\t%3.3f\t(%d)\tphi = [%3.2f]\tdt_phi = [%3.2f]\n",
						rt_step_i,
						t_ref,
						(int)up_time_end - (int)up_time,
						phi_ref,
						dt_phi_ref);

					printf("   p_ref   = [%3.3f, %3.3f]\tdt_p_ref = [%3.3f, %3.3f]\n",
						p_ref[IDX_X],
						p_ref[IDX_Y],
						dt_p_ref[IDX_X],
						dt_p_ref[IDX_Y]);

					/*
					printf("   p_m     = [%3.3f, %3.3f]\tdt_p_m   = [%3.3f, %3.3f]\tF_m = [%3.3f, %3.3f]\n",
						p_m[IDX_X],
						p_m[IDX_Y],
						dt_p_m[IDX_X],
						dt_p_m[IDX_Y],
						F_end_m[IDX_X],
						F_end_m[IDX_Y]);
					*/

					/*
					printf("   F_fb_x = [%3.2f]\tF_ff_x = [%3.2f]\tF_gc_x = [%3.2f]\tF_end_cmd_x = [%3.2f\t(%3.2f)]\n",
							F_end_cmd_fb[IDX_X], F_end_cmd_ff[IDX_X] , F_end_cmd_gcomp[IDX_X], F_end_cmd[IDX_X], LL_motors_settings.force_end[IDX_X]);
					printf("   F_fb_y = [%3.2f]\tF_ff_y = [%3.2f]\tF_gc_y = [%3.2f]\tF_end_cmd_y = [%3.2f\t(%3.2f)]\n",
							F_end_cmd_fb[IDX_Y], F_end_cmd_ff[IDX_Y] , F_end_cmd_gcomp[IDX_Y], F_end_cmd[IDX_Y], LL_motors_settings.force_end[IDX_Y]);

					printf("   current[LEFT, RIGHT] = [%3.3f, %3.3f] \n", LL_motors_settings.left.current, LL_motors_settings.right.current);
					printf("   volt[LEFT, RIGHT]    = [%3.3f, %3.3f] \n", LL_motors_settings.left.volt,    LL_motors_settings.right.volt);
					printf("   dac_in[LEFT, RIGHT]  = [%i, %i] \n",       LL_motors_settings.left.dac_in,  LL_motors_settings.right.dac_in);
					*/

					/*
					printf("   calib_enc_on = [%d], cmd_code_prev = [%s], cmd_code = [%s] \n",
							calib_enc_on, CMD_STR[cmd_code_prev], CMD_STR[cmd_code]);
					printf("   lowerlimb_sys_info.activity_state = [%s] \n",   ACTIV_STATE_STR[lowerlimb_sys_info.activity_state]);
					printf("\n");
					*/
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			// Increase real-time step counter:
			///////////////////////////////////////////////////////////////////////////////

			rt_step_i++;

		} // end if (up_time >= algo_nextTime) // RT time step
	} // end while (t_ref <= T_RUN_MAX)
}
