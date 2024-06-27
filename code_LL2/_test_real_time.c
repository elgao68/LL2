/////////////////////////////////////////////////////////////////////////////
//
//  _test_real_time.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <_test_real_time.h>

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// TEST SCRIPT - REAL-TIME - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_real_time(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3) {

	///////////////////////////////////////////////////////////////////////////////
	// MOTOR STATE VARS - GAO:
	///////////////////////////////////////////////////////////////////////////////

	uint8_t motor_result        = 0;
	uint8_t motor_alert         = 0;

	uint8_t motor_torque_active = 0;

	///////////////////////////////////////////////////////////////////////////////
	// FIRMWARE/CONTROL PARAMETERS - GAO:
	///////////////////////////////////////////////////////////////////////////////

	uint64_t up_time;
	uint64_t up_time_end;

	lowerlimb_mech_readings_t   LL_mech_readings;
	lowerlimb_motors_settings_t LL_motors_settings;
	traj_ctrl_params_t          traj_ctrl_params;
	admitt_model_params_t       admitt_model_params;
	lowerlimb_ref_kinematics_t	ref_kinematics;

	const int is_calibration = 1; // what did this flag do in update_motor_algo() (now set_LL_mech_readings())?

	///////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - REFERENCE:
	///////////////////////////////////////////////////////////////////////////////

    // Integrator time step:
	double dt_k = (double)DT_STEP_MSEC/MSEC_PER_SEC; // integrator step (initial)

	// Reference position and velocity:
	double    p_ref[N_COORD_2D] = {0.0, 0.0};
	double dt_p_ref[N_COORD_2D] = {0.0, 0.0};

	// Cycle phase and instantaneous frequency:
	double    phi_ref = 0.0;
	double dt_phi_ref = 0.0;

	// Trajectory path tangent vector:
	double u_t_ref[N_COORD_2D] = {0.0, 0.0};

	// Internal state: initial values:
	double z_intern_o_dbl[2*N_COORD_EXT] = {0.0, 0.0, PHI_INIT, 0.0, 0.0, 0.0};

	// Calibration end points:
	double p_calib_o[N_COORD_2D] = {0.0, 0.0};
	double p_calib_f[N_COORD_2D] = {0.0, 0.0};

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
	double OMEGA_HI_ERR_INT_POS         = 2*PI*0.2;

	double err_pos[N_COORD_2D]          = {0.0, 0.0}; // HACK: this term should have been avoided
	double err_int_pos[N_COORD_2D]      = {0.0, 0.0};
	double err_int_pos_prev[N_COORD_2D] = {0.0, 0.0};

	double scale_ff_dyn      = 0.0; // FF control: dynamic scaling
	double F_end_cmd_ff_norm = 0.0;

	///////////////////////////////////////////////////////////////////////////////
	// Calibration variables:
	///////////////////////////////////////////////////////////////////////////////

	double T_f_calib   = 0.0;

	double t_calib     = 0.0;
	double t_ref_calib = 0.0;

	calib_state_t calib_state = CalibStateNull;
	uint8_t init_calib  = 1;

	// Calibration states:
	uint8_t calib_fsens_on = 0;
	uint8_t calib_enc_on   = 0;

	///////////////////////////////////////////////////////////////////////////////
	// TCP/IP variables:
	///////////////////////////////////////////////////////////////////////////////

	int sock_status;

	///////////////////////////////////////////////////////////////////////////////
	// State variables:
	///////////////////////////////////////////////////////////////////////////////

	uint16_t cmd_code  = 0, cmd_code_prev_to_last = 0;
	uint8_t  app_state = 0;
	bool brake_cmd;

	uint8_t system_state_prev   = LL_sys_info.system_state;
	uint8_t activity_state_prev = LL_sys_info.activity_state;
	uint8_t exercise_state_prev = LL_sys_info.exercise_state;

	static uint8_t traj_type    = NullTraj;

	///////////////////////////////////////////////////////////////////////////////
	// Real-time counters, timers and switches:
	///////////////////////////////////////////////////////////////////////////////

	double T_RUN_MAX    = 5000;
	double t_ref        = 0.0;

	int rt_step_i       = 0; // real-time step counter
	int r_i, c_i, v_i; // general-purpose counters

	int8_t switch_traj  = SWITCH_TRAJ_NULL;

	uint8_t init_traj_params = 1;

	///////////////////////////////////////////////////////////////////////////////
	// Monitoring variables:
	///////////////////////////////////////////////////////////////////////////////

	// TCP communication checks:
	int32_t ret_tcp_msg      = 0;
	uint64_t dt_ret_tcp_msec = 0;

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
		printf("test_real_time():\n\n");
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
			// GET TCP/IP APP STATE - GAO
			//////////////////////////////////////////////////////////////////////////////////

			#if USE_APP_TCP_IP
				// Template function for the firmware state machine:
				LL_sys_info = lowerlimb_app_state_tcpip(Read_Haptic_Button(), motor_alert,
						&traj_ctrl_params, &admitt_model_params, &LL_motors_settings, &cmd_code,
						&calib_enc_on);
			#else
				LL_sys_info = lowerlimb_app_state(Read_Haptic_Button(), motor_alert,
						&traj_ctrl_params, &admitt_model_params, &LL_motors_settings, &cmd_code);
			#endif

			// HACK: override SETUP state
			if (LL_sys_info.exercise_state == SETUP)
				LL_sys_info.exercise_state = RUNNING;

			///////////////////////////////////////////////////////////////////////////////
			// Clear motor_alert after sending it to TCP/IP APP state:
			///////////////////////////////////////////////////////////////////////////////

			motor_alert = 0;

			//////////////////////////////////////////////////////////////////////////////////
			// Override trajectory and control parameters - TODO: remove at a later date
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

				static double damp_ratio = DAMP_RATIO_DEF;
				static double w_n        = OMEGA_N_DEF; // natural frequency
				static double sigma;

				sigma = damp_ratio*w_n;

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

				if (init_traj_params) {
					if (traj_ctrl_params.semiaxis_x == 0 &&	traj_ctrl_params.semiaxis_y == 0)
						 traj_type = IsometricTraj;
					else if (traj_ctrl_params.semiaxis_x != 0 && traj_ctrl_params.semiaxis_y == 0)
						 traj_type = LinearTraj;
					else
						 traj_type = EllipticalTraj;

					#if USE_ITM_OUT_RT_CHECK
						printf("   test_real_time: traj_type = [%s]\n\n", TRAJ_TYPE_STR[traj_type]);
					#endif

					init_traj_params = 0;
				}
			#endif

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

			if (LL_sys_info.system_state == SYS_ON) {

				///////////////////////////////////////////////////////////////////////////////
				// Update LEDs state:
				///////////////////////////////////////////////////////////////////////////////

				cycle_haptic_buttons();

				///////////////////////////////////////////////////////////////////////////////
				// Update safety:
				///////////////////////////////////////////////////////////////////////////////

				set_safetyOff(LL_sys_info.safetyOFF);

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

				if (exercise_state_prev != RUNNING && LL_sys_info.exercise_state == RUNNING)
					switch_traj = SWITCH_TRAJ_START;
				else if ((exercise_state_prev != SLOWING && LL_sys_info.exercise_state == SLOWING) ||
						 (exercise_state_prev != STOPPED && LL_sys_info.exercise_state == STOPPED))
					switch_traj = SWITCH_TRAJ_END;
				else
					switch_traj = SWITCH_TRAJ_NULL;

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// ACTIVITY STATE SWITCH:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				// IDLE state:
				///////////////////////////////////////////////////////////////////////////////

				if (LL_sys_info.activity_state == IDLE) {

					// Clear motor algo readings and settings:
					clear_lowerlimb_mech_readings(&LL_mech_readings);
					clear_lowerlimb_motors_settings(&LL_motors_settings);
					clear_ctrl_params();

					// Disable motors:
					motor_L_move(0, false, false);
					motor_R_move(0, false, false);

					// Restart motor algorithm:
					init_motor_algo(&LL_mech_readings, &LL_motors_settings);

					// Reset calibration state:
					// reset_calibration_state();

				} // end if (LL_sys_info.activity_state == IDLE)

				///////////////////////////////////////////////////////////////////////////////
				// CALIBRATION state:
				///////////////////////////////////////////////////////////////////////////////

				else if (LL_sys_info.activity_state == CALIB) {

					#if USE_ITM_OUT_RT_CHECK
						if (cmd_code != cmd_code_prev_to_last) {
							printf("   test_real_time(): cmd_code_prev = [%s], cmd_code = [%s]\n\n", CMD_STR[cmd_code_prev_to_last], CMD_STR[cmd_code]);
						}
					#endif

					if (calib_state == CalibStateNull) {

						calib_state = CalibStateTraj_1;

						t_ref_calib = t_ref;
						init_calib  = 1;

						p_calib_o[IDX_X] = p_m[IDX_X];
						p_calib_o[IDX_Y] = p_m[IDX_Y];

						p_calib_f[IDX_X] = p_calib_o[IDX_X] + SEMIAXIS_X_DEF;
						p_calib_f[IDX_Y] = p_calib_o[IDX_Y];

						#if USE_ITM_OUT_RT_CHECK
							printf("   test_real_time(): calib_state  = [%s] \n", CALIB_STATE_STR[CalibStateTraj_1]);
							printf("                     calib_enc_on = [%d] \n\n", calib_enc_on);
						#endif
					}

					// Calibration timer:
					if (calib_state == CalibStateTraj_1 || calib_state == CalibStateTraj_2)
						t_calib = t_ref - t_ref_calib;
					else
						t_calib = 0;

					// Generate trajectory points:
					traj_linear_points(	p_ref, dt_p_ref, u_t_ref, dt_k,
										p_calib_o, p_calib_f, V_CALIB, ALPHA_CALIB, &init_calib, &T_f_calib);

					if (t_calib >= T_f_calib)
						calib_enc_on = 0;

					// Reset encoders:
					/*
					qei_count_L_reset();
					qei_count_R_reset();
					*/
				} // end if (LL_sys_info.activity_state == CALIB)

				///////////////////////////////////////////////////////////////////////////////
				// JOG state:
				///////////////////////////////////////////////////////////////////////////////

				else if (LL_sys_info.activity_state == JOG) {

				} // end if (LL_sys_info.activity_state == JOG)

				///////////////////////////////////////////////////////////////////////////////
				// EXERCISE state:
				///////////////////////////////////////////////////////////////////////////////

				else if (LL_sys_info.activity_state == EXERCISE) {

					///////////////////////////////////////////////////////////////////////////////
					// EXERCISE STATE SWITCH:
					///////////////////////////////////////////////////////////////////////////////

					if (LL_sys_info.exercise_state == RUNNING || LL_sys_info.exercise_state == SLOWING) {

						///////////////////////////////////////////////////////////////////////////////
						// Generate reference trajectory:
						///////////////////////////////////////////////////////////////////////////////

						// Passive trajectory control:
						if (LL_sys_info.exercise_mode == PassiveTrajectoryCtrl) {
							if (traj_type == EllipticalTraj || traj_type == LinearTraj) {
								traj_ref_step_passive_elliptic(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref, dt_k,
									traj_ctrl_params, switch_traj, TRAJ_PARAMS_VARIABLE_ON);
							}
							else
								// Isometric trajectory OR safety catch:
								traj_ref_step_isometric(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref);
						}

						// Active trajectory control:
						else if (LL_sys_info.exercise_mode == ActiveTrajectoryCtrl) {
							if (traj_type == EllipticalTraj)
								traj_ref_step_active_elliptic(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref, dt_k, F_end_in, z_intern_o_dbl,
									traj_ctrl_params, admitt_model_params, ADMITT_MODEL_CONSTR_ON, switch_traj, TRAJ_PARAMS_VARIABLE_ON);

							else if (traj_type == LinearTraj)
								traj_ref_step_active_elliptic(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref, dt_k, F_end_in, z_intern_o_dbl,
									traj_ctrl_params, admitt_model_params, ADMITT_MODEL_CONSTR_ON, switch_traj, TRAJ_PARAMS_VARIABLE_OFF);

							else
								// Isometric trajectory OR safety catch:
								traj_ref_step_isometric(
									p_ref, dt_p_ref,
									&phi_ref, &dt_phi_ref,
									u_t_ref);
						}

						// Undefined exercise mode:
						else {
							#if USE_ITM_OUT_RT_CHECK
								printf("   exercise_mode: [%s]\n", EXERC_MODE_STR[LL_sys_info.exercise_mode]);
							#endif
						}

					} // end if (LL_sys_info.exercise_state == RUNNING)
					// end EXERCISE STATE SWITCH

				} // end if (LL_sys_info.activity_state == EXERCISE)

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// MOTOR COMMANDS GENERATION:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				// Set reference kinematics struct:
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

				if ( LL_sys_info.activity_state == CALIB ||
					(LL_sys_info.activity_state == EXERCISE && (LL_sys_info.exercise_state == RUNNING || LL_sys_info.exercise_state == SLOWING)) ) {

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
					F_end_cmd_gcomp[IDX_Y] = SCALE_GCOMP*F_G_COMP_DEF + F_gcomp_dyn;

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
					nml_mat_smult_r(F_end_cmd_fb_nml, SCALE_FB); // scale by switching variable

					// FB force commands, array form:
					for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
						F_end_cmd_fb[c_i] = F_end_cmd_fb_nml->data[c_i][0];

					///////////////////////////////////////////////////////////////////////////////
					// FF force command:
					///////////////////////////////////////////////////////////////////////////////

					// FF force commands, array form:
					for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
						F_end_cmd_ff[c_i] = SCALE_FF*C_FF_DC_DEF[c_i]*dt_p_ref[c_i];

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
						F_end_cmd_int_err[c_i] = K_INT_ERR_POS*err_int_pos[c_i];

					///////////////////////////////////////////////////////////////////////////////
					// Total force command:
					///////////////////////////////////////////////////////////////////////////////

					for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
						F_end_cmd[c_i] = F_end_cmd_fb[c_i] + F_end_cmd_ff[c_i] + F_end_cmd_gcomp[c_i] + F_end_cmd_int_err[c_i];

					///////////////////////////////////////////////////////////////////////////////
					///////////////////////////////////////////////////////////////////////////////
					// UPDATE MOTOR SETTINGS - GAO:
					///////////////////////////////////////////////////////////////////////////////
					///////////////////////////////////////////////////////////////////////////////

					// Clear motors' settings:
					clear_lowerlimb_motors_settings(&LL_motors_settings);

					// Compute motors' settings:
					set_LL_motor_settings(&LL_motors_settings, F_end_cmd);

					///////////////////////////////////////////////////////////////////////////////
					// Send motor commands:
					///////////////////////////////////////////////////////////////////////////////

					// Check motor activation status:
					if (	(LL_sys_info.activity_state == CALIB    && MOTOR_TORQUE_ACTIVE_CALIB) ||
							(LL_sys_info.activity_state == JOG      && MOTOR_TORQUE_ACTIVE_JOG) ||
							(LL_sys_info.activity_state == EXERCISE && MOTOR_TORQUE_ACTIVE_EXERCISE) )
						motor_torque_active = 1;
					else
						motor_torque_active = 0;

					// Issue motion commands to motors::
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
						printf("\n");
						printf("   MOTOR ALERT = [%d] \n\n", motor_alert);
					}
				}
				else {
					// Reset sensor readings and motor settings:
					clear_lowerlimb_mech_readings(&LL_mech_readings);
					clear_lowerlimb_motors_settings(&LL_motors_settings);
					clear_ctrl_params();

					// Set the motors to 0 and disable the motor driver
					motor_L_move(0, false, false);
					motor_L_move(0, false, false);
				}

				///////////////////////////////////////////////////////////////////////////////
				// Send feedback data to TCP/IP app:
				///////////////////////////////////////////////////////////////////////////////

				// HACK: include low-pass filtered force sensor measurements in robot readings:
				LL_mech_readings.Xforce = F_end_lo[IDX_X];
				LL_mech_readings.Yforce = F_end_lo[IDX_Y];

				// HACK: store integral position error temporarily in LL_mech_readings (for debugging):
				/*
				LL_mech_readings.velocity.x	= err_int_pos[IDX_X];
				LL_mech_readings.velocity.y = err_int_pos[IDX_Y];
				*/

				send_lowerlimb_exercise_feedback(up_time, &LL_mech_readings, &LL_motors_settings, &ref_kinematics);

				///////////////////////////////////////////////////////////////////////////////
				// ITM console output:
				///////////////////////////////////////////////////////////////////////////////

				#if USE_ITM_OUT_RT_CHECK
					/*
					if ((up_time_end - up_time) > DT_STEP_MSEC) {
						printf("____________________________\n");
						printf("rt_step_i [%d]: t_ref = %f\tDT MSEC = %d\n",
							rt_step_i,
							t_ref,
							(int)up_time_end - (int)up_time;
						printf("____________________________\n");
					}
					*/

					if (switch_traj != SWITCH_TRAJ_NULL) {
						// Check uptime after computations:
						up_time_end = getUpTime();

						printf("   switch_traj = [%d] \n\n", switch_traj);
						printf("   %d\t%3.3f\t(%d)\tphi = [%3.2f]\tdt_phi = [%3.2f]\tp_ref = [%3.3f, %3.3f]\tdt_p_ref = [%3.3f, %3.3f], p_m = [%3.3f, %3.3f]\tdt_p_m = [%3.3f, %3.3f]\tF_m = [%3.3f, %3.3f]\n",
							rt_step_i,
							t_ref,
							(int)up_time_end - (int)up_time,
							phi_ref,
							dt_phi_ref,
							p_ref[IDX_X],
							p_ref[IDX_Y],
							dt_p_ref[IDX_X],
							dt_p_ref[IDX_Y],
							p_m[IDX_X],
							p_m[IDX_Y],
							dt_p_m[IDX_X],
							dt_p_m[IDX_Y],
							F_end_m[IDX_X],
							F_end_m[IDX_Y]);
					}
				#endif
			} // end if (LL_sys_info.system_state == ON)

			else { // LL_sys_info.system_state != ON
				// Change LEDs to system state OFF:
				LED_sys_state_off();

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

			system_state_prev   = LL_sys_info.system_state;
			activity_state_prev = LL_sys_info.activity_state;
			exercise_state_prev = LL_sys_info.exercise_state;

			///////////////////////////////////////////////////////////////////////////////
			// ITM console output:
			///////////////////////////////////////////////////////////////////////////////

			#if USE_ITM_OUT_RT_CHECK_LONG
				if (rt_step_i % (DT_DISP_MSEC_REALTIME/DT_STEP_MSEC) == 0) {
					// Check uptime after computations:
					up_time_end = getUpTime();

					printf("   %d\t%3.3f\t(%d)\tphi = [%3.2f]\tdt_phi = [%3.2f]\tp_ref = [%3.3f, %3.3f]\tdt_p_ref = [%3.3f, %3.3f], p_m = [%3.3f, %3.3f]\tdt_p_m = [%3.3f, %3.3f]\tF_m = [%3.3f, %3.3f]\n",
						rt_step_i,
						t_ref,
						(int)up_time_end - (int)up_time,
						phi_ref,
						dt_phi_ref,
						p_ref[IDX_X],
						p_ref[IDX_Y],
						dt_p_ref[IDX_X],
						dt_p_ref[IDX_Y],
						p_m[IDX_X],
						p_m[IDX_Y],
						dt_p_m[IDX_X],
						dt_p_m[IDX_Y],
						F_end_m[IDX_X],
						F_end_m[IDX_Y]);

					printf("   \t\t\t\t\tF_fb_x = [%3.2f]\tF_ff_x = [%3.2f]\tF_gc_x = [%3.2f]\tF_end_cmd_x = [%3.2f\t(%3.2f)]\n",
							F_end_cmd_fb[IDX_X], F_end_cmd_ff[IDX_X] , F_end_cmd_gcomp[IDX_X], F_end_cmd[IDX_X], LL_motors_settings.force_end[IDX_X]);
					printf("   \t\t\t\t\tF_fb_y = [%3.2f]\tF_ff_y = [%3.2f]\tF_gc_y = [%3.2f]\tF_end_cmd_y = [%3.2f\t(%3.2f)]\n",
							F_end_cmd_fb[IDX_Y], F_end_cmd_ff[IDX_Y] , F_end_cmd_gcomp[IDX_Y], F_end_cmd[IDX_Y], LL_motors_settings.force_end[IDX_Y]);

					// printf("   \t\t\t\t\tcurrent[LEFT, RIGHT] = [%3.3f, %3.3f] \n", LL_motors_settings.left.current, LL_motors_settings.right.current);
					// printf("   \t\t\t\t\tvolt[LEFT, RIGHT]    = [%3.3f, %3.3f] \n", LL_motors_settings.left.volt,    LL_motors_settings.right.volt);

					printf("   \t\t\t\t\tdac_in[LEFT, RIGHT]  = [%i, %i] \n",         LL_motors_settings.left.dac_in,  LL_motors_settings.right.dac_in);
					printf("   \t\t\t\t\tt_calib  = [%3.2f], T_f_calib = [%3.2f], calib_enc_on = [%d], cmd_code_prev_to_last = [%s], cmd_code = [%s] \n",
							t_calib, T_f_calib, calib_enc_on, CMD_STR[cmd_code_prev_to_last], CMD_STR[cmd_code]);
					printf("   \t\t\t\t\tLL_sys_info.activity_state = [%s] \n",       ACTIV_STATE_STR[LL_sys_info.activity_state]);
					printf("\n");
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			// Increase real-time step counter:
			///////////////////////////////////////////////////////////////////////////////

			// Check command code values:
			cmd_code_prev_to_last = cmd_code;

			rt_step_i++;

		} // end if (up_time >= algo_nextTime) // RT time step
	} // end while (t_ref <= T_RUN_MAX)
}
