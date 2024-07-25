/////////////////////////////////////////////////////////////////////////////
//
//  _test_real_time_statemach_control.c
//
// Created on: 2024.07.10
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
test_real_time_statemach_control(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3) {

	///////////////////////////////////////////////////////////////////////////////
	// Select TCP command codes list (CRITICAL):
	///////////////////////////////////////////////////////////////////////////////

	__SELECT_CMD_CODE_LIST(USE_SOFTWARE_MSG_LIST)

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

	#if USE_ITM_OUT_RT_CHECK_CTRL
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
	double z_intern_home_dbl[2*N_COORD_EXT];

	double    phi_home = PHI_HOME;
	double dt_phi_home = 0.0;

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

	uint8_t idx_scale_gain = IDX_SCALE_GAIN_EXERCISE; // scale arrays selector, activity-based (CRITICAL)

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

	// Exercise mode - SLOWING:
	double t_slow     = 0.0; // TODO: consider using an implementation that doesn't require an explicit time reference (local step counts?)
	double t_slow_ref = 0.0;

	///////////////////////////////////////////////////////////////////////////////
	// TCP/IP communications variables:
	///////////////////////////////////////////////////////////////////////////////

	int sock_status;

	// TCP communication checks:
	int32_t ret_tcp_msg      = 0;
	uint64_t dt_ret_tcp_msec = 0;

	///////////////////////////////////////////////////////////////////////////////
	// TCP/IP message PAYLOAD variables:
	///////////////////////////////////////////////////////////////////////////////

	// Void pointer to TCP message PAYLOAD - should be recast to the correct data type depending on the payload (CRITICAL):
	void* msg_payload = &tcpRxData[payloadStart_index];

	// Payload variables (message specific):
	exercise_mode_t exercise_mode;

	///////////////////////////////////////////////////////////////////////////////
	// Firmware state & command code variables:
	///////////////////////////////////////////////////////////////////////////////

	// Command codes - TCP:
	uint16_t cmd_code_tcp         = NO_CMD;
	uint8_t is_valid_cmd_code_tcp = 0;

	// Internal firmware messages:
	uint16_t msg_code_intern     = NO_CMD;

	// Firmware state - CRITICAL:
	uint16_t state_fw         = ST_FW_SYSTEM_OFF;
	uint16_t state_fw_prev    = state_fw;
	uint8_t state_fw_changed  = 0;

	///////////////////////////////////////////////////////////////////////////////
	// Firmware modes / modes:
	///////////////////////////////////////////////////////////////////////////////

	// CALIBRATION modes:
	calib_traj_t calib_traj   = CalibTraj_Null;
	uint8_t calib_enc_traj_on = 0;

	// HOMING modes:
	double OMEGA_THR_HOMING_START = 0.03;
	uint8_t homing_traj_on        = 0; // CRITICAL initialization

	// EXERCISE modes:
	uint8_t pace_exerc      = RUNNING;
	uint8_t pace_exerc_prev = pace_exerc;
	uint8_t init_traj_exerc = 0;

	// Trajectory modes:
	int8_t mode_traj = MODE_TRAJ_NULL; // NOTE: this is a SIGNED integer

	///////////////////////////////////////////////////////////////////////////////
	// Real-time counters and timers:
	///////////////////////////////////////////////////////////////////////////////

	double T_RUN_MAX = 5000;
	double t_ref     = 0.0;

	int rt_step_i    = 0; // real-time step counter
	int r_i, c_i; // general-purpose counters

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

	#if USE_ITM_OUT_RT_CHECK_CTRL
		printf("test_real_time_statemach_control():\n\n");
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
					printf("rt_step_i [%d]: cmd_code_tcp = [%d], ret_tcp_msg = [%d], dt_ret_tcp_msec = [%d]\n\n", rt_step_i, cmd_code_tcp, ret_tcp_msg, (int)dt_ret_tcp_msec);
				}
			#endif

			//////////////////////////////////////////////////////////////////////////////////
			// Get TCP/IP message:
			//////////////////////////////////////////////////////////////////////////////////

			is_valid_cmd_code_tcp =
					is_valid_rcv_data_cmd_code(&cmd_code_tcp, Read_Haptic_Button(), motor_alert,
							USE_SOFTWARE_MSG_LIST, &lowerlimb_sys_info, tcpRxData);

			// Clear motor_alert (TODO: what exactly does this do)?
			motor_alert = 0;

			#if USE_ITM_CMD_DISPLAY
				if (is_valid_cmd_code_tcp) {
					printf("   --------------------------------------\n");
					printf("   cmd_code_tcp(%d) [%s] (state machine TCP app) \n\n", cmd_code_tcp, __CMD_CODE_STR(cmd_code_tcp));
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			// ITM Console output:
			///////////////////////////////////////////////////////////////////////////////

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
			// PAYLOAD extraction for different command codes (TODO: complete this section!):
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			if (is_valid_cmd_code_tcp) {
				if (cmd_code_tcp == _START_SYSTEM) { }
				else if (cmd_code_tcp == _BRAKES_ON_OFF	) { }
				else if (cmd_code_tcp == _CALIBRATE) { }

				else if (cmd_code_tcp == _START_EXERCISE) {
					// Obtain exercise mode from payload:
					exercise_mode = *(exercise_mode_t*)msg_payload;

					#if USE_ITM_EXERC_MODE_CHECK
						printf("   <<test_real_time_statemach_control>> cmd_code (%d) [%s]: PAYLOAD exercise_mode (%d) [%s]\n\n",
								cmd_code_tcp, __CMD_CODE_STR(cmd_code_tcp), exercise_mode, EXERC_MODE_STR[exercise_mode]);
					#endif
				}

				else if (cmd_code_tcp == _STOP_EXERCISE) { }
				else if (cmd_code_tcp == _STOP_SYSTEM) { }
				else if (cmd_code_tcp == _MOVE_TO_START) { }
				else if (cmd_code_tcp == _GO_TO_EXERCISE) { }
				else if (cmd_code_tcp == _PEDAL_TRAVEL) { }
				else if (cmd_code_tcp == _FORCE_THERAPY_CHANGE) { }
				else if (cmd_code_tcp == _STDBY_START_POINT) { }
			}

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// STATE MACHINE for TCP app commands:
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			state_machine_ll2(&state_fw, &cmd_code_tcp, &msg_code_intern, &pace_exerc); // USE_SOFTWARE_MSG_LIST

			// Register change of state (CRITICAL):
			state_fw_changed = (state_fw != state_fw_prev);

			///////////////////////////////////////////////////////////////////////////////
			// ITM Console output:
			///////////////////////////////////////////////////////////////////////////////

			#if USE_ITM_CMD_DISPLAY
				if (state_fw_changed || rt_step_i == 0) {
					printf("______________________________________\n");
					printf("FIRMWARE STATE (%d) [%s] (TCP app) \n\n",	state_fw, STR_ST_FW[state_fw - OFFS_ST_FW]); // pace_exerc, PACE_EXERC_STR[pace_exerc]
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// STATE-BASED ACTIONS (1): INITIAL
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Actions for multiple states:
			///////////////////////////////////////////////////////////////////////////////

			if (rt_step_i == 0 || state_fw_changed) {

				///////////////////////////////////////////////////////////////////////////////
				// State-dependent gains scaling index - defaults - CRITICAL:
				///////////////////////////////////////////////////////////////////////////////

				if (state_fw != ST_FW_CALIBRATING_AND_HOMING)
					idx_scale_gain = IDX_SCALE_GAIN_EXERCISE;

				///////////////////////////////////////////////////////////////////////////////
				// Motor torque activation status - CRITICAL:
				///////////////////////////////////////////////////////////////////////////////

				#if (MODE_STANDALONE_BOARD == 0)
					if (state_fw == ST_FW_SYSTEM_OFF || state_fw == ST_FW_CONNECTING)
						motor_torque_active = 0;
					else if (state_fw == ST_FW_CALIBRATING_AND_HOMING)
						motor_torque_active = 0;
					else if (state_fw == ST_FW_HOMING)
						motor_torque_active = 0;
					else if (state_fw == ST_FW_ADJUST_EXERCISE)
						motor_torque_active = 0;
					else if (state_fw == ST_FW_EXERCISE_ON)
						motor_torque_active = 0;
					else if (state_fw == ST_FW_STDBY_AT_POSITION)
						motor_torque_active = 0;
				#else
					motor_torque_active = 0;
				#endif
			}

			///////////////////////////////////////////////////////////////////////////////
			// State: SYSTEM_OFF
			///////////////////////////////////////////////////////////////////////////////

			// Actions for inactive system:
			if (state_fw == ST_FW_SYSTEM_OFF) { //
				if (rt_step_i == 0 || state_fw_changed) {
					// Change LEDs to system state OFF:
					LED_sys_state_off();

					// Disable motors:
					motor_L_move(0, false, false);
					motor_R_move(0, false, false);

					// Engage brakes:
					if (!OVERR_BRAKES) {
						l_brakes(ENGAGE_BRAKES);
						r_brakes(ENGAGE_BRAKES);
					}
					else {
						l_brakes(DISENGAGE_BRAKES);
						r_brakes(DISENGAGE_BRAKES);
					}

					// Safety catch: reset integral error
					err_int_pos[IDX_X] = 0;
					err_int_pos[IDX_Y] = 0;
				}
			}

			///////////////////////////////////////////////////////////////////////////////
			// State: (!SYSTEM_OFF)
			///////////////////////////////////////////////////////////////////////////////

			// Actions for active system:
			else {

				///////////////////////////////////////////////////////////////////////////////
				// Update safety:
				///////////////////////////////////////////////////////////////////////////////

				set_safetyOff(lowerlimb_sys_info.safetyOFF);

				///////////////////////////////////////////////////////////////////////////////
				// State: CALIBRATING
				///////////////////////////////////////////////////////////////////////////////

				if (state_fw == ST_FW_CALIBRATING_AND_HOMING) {

					// Force sensors calibration & initial encoder resets - CRITICAL: this should always happen before SENSOR readings
					if (state_fw_changed) {
						// Zero-calibrate force sensors:
						force_sensors_zero_calibrate(hadc3);

						// Reset encoders - CRITICAL:
						qei_count_L_reset();
						qei_count_R_reset();

						// Activate encoder calibration:
						calib_enc_traj_on = 1;

						#if USE_ITM_CMD_DISPLAY
							printf("   <<test_real_time_statemach_control()>> [Force sensors calibrated] \n\n");
						#endif
					}
				}

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

			} // end if (state_fw != ST_FW_SYSTEM_OFF) - INITIAL

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// STATE-BASED ACTIONS (2): INTERMEDIATE - most of the work happens here
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// State: CONNECTING
			///////////////////////////////////////////////////////////////////////////////

			if (state_fw == ST_FW_CONNECTING) {
				if (state_fw_changed) {
					// Update LEDs state:
					LED_sys_state_on(); // TODO: was cycle_haptic_buttons();

					// Disengage brakes:
					l_brakes(DISENGAGE_BRAKES);
					r_brakes(DISENGAGE_BRAKES);
				}
			}

			///////////////////////////////////////////////////////////////////////////////
			// State: STANDBY
			///////////////////////////////////////////////////////////////////////////////

			else if (state_fw == ST_FW_STDBY_AT_POSITION) {
				if (state_fw_changed) {
					// Generate HOME point:
					home_point_ellipse(phi_home, dt_phi_home, p_ref, dt_p_ref, &traj_ctrl_params);

					dt_p_ref[IDX_X] = 0.0;
					dt_p_ref[IDX_Y] = 0.0;
				}
			}

			///////////////////////////////////////////////////////////////////////////////
			// State: CALIBRATING
			///////////////////////////////////////////////////////////////////////////////

			else if (state_fw == ST_FW_CALIBRATING_AND_HOMING) {

				// Generate calibration trajectory (NOTE: calib_enc_traj_on can only be zero'd by this call):
				traj_ref_calibration_ll2(
					p_ref, dt_p_ref, &calib_enc_traj_on, &calib_traj, &idx_scale_gain,
					dt_k, p_m, dt_p_m, phi_home, dt_phi_home,
					&LL_motors_settings, &traj_ctrl_params, traj_exerc_type, V_CALIB, FRAC_RAMP_CALIB);

				// If motor torque is inactive, consider encoder calibration completed (only for testing):
				if (MODE_STANDALONE_BOARD || !motor_torque_active)
					calib_enc_traj_on = 0;

				// Signal end of calibration trajectory:
				if (!calib_enc_traj_on) {
					// Internal message: calibration completed
					msg_code_intern = CALIB_ENC_COMPLETED_CMD;

					#if USE_ITM_CMD_DISPLAY
						printf("   <<test_real_time_statemach_control()>> [Encoders calibrated] \n\n");
					#endif
				}
			}

			///////////////////////////////////////////////////////////////////////////////
			// State: HOMING
			///////////////////////////////////////////////////////////////////////////////

			else if (state_fw == ST_FW_HOMING) {
				if (state_fw_changed) {
					// Activate homing:
					homing_traj_on = 1;

					#if USE_ITM_OUT_RT_CHECK_CTRL
						printf("   <<test_real_time_statemach_control()>> homing_traj_on = [%d] \n\n", homing_traj_on);
					#endif
				}

				// Generate homing trajectory:
				traj_ref_homing_ll2(p_ref, dt_p_ref, &homing_traj_on, &idx_scale_gain,
					dt_k, p_m, dt_p_m, phi_home, dt_phi_home,
					&traj_ctrl_params, V_CALIB, FRAC_RAMP_CALIB);

				// Signal end of homing trajectory:
				if (!homing_traj_on)
					// Internal message : homing completed
					msg_code_intern = HOMING_COMPLETED_CMD;
			}

			///////////////////////////////////////////////////////////////////////////////
			// State: EXERCISE
			///////////////////////////////////////////////////////////////////////////////

			else if (state_fw == ST_FW_EXERCISE_ON) {
				if (state_fw_changed) {
					// Reset emergency alerts:
					reset_emergency_alerts(); // TODO: leftover call - what does this do?

					// Initialize internal state for active trajectory control (CRITICAL):
					home_point_ellipse(phi_home, dt_phi_home, p_ref, dt_p_ref, &traj_ctrl_params);

					z_intern_home_dbl[IDX_X  ]    = p_ref[IDX_X];
					z_intern_home_dbl[IDX_Y  ]    = p_ref[IDX_Y];
					z_intern_home_dbl[IDX_PHI]    = phi_home;

					z_intern_home_dbl[IDX_DT_X  ] = 0;
					z_intern_home_dbl[IDX_DT_Y  ] = 0;
					z_intern_home_dbl[IDX_DT_PHI] = dt_phi_home;

					// Initial trajectory flag (CRITICAL):
					init_traj_exerc = 1;
				}
				else if (pace_exerc_prev != SLOWING && pace_exerc == SLOWING)
					t_slow_ref  = t_ref;

				///////////////////////////////////////////////////////////////////////////////
				// Generate reference trajectory:
				///////////////////////////////////////////////////////////////////////////////

				// Trajectory mode:
				if (pace_exerc != SLOWING)
					mode_traj = MODE_TRAJ_START;
				else
					mode_traj = MODE_TRAJ_END;

				// Passive trajectory control:
				if (exercise_mode == PassiveTrajectoryCtrl) {
					if (traj_exerc_type == EllipticTraj || traj_exerc_type == LinearTraj)
						traj_ref_step_passive_elliptic(
							phi_home,
							p_ref, dt_p_ref,
							&phi_ref, &dt_phi_ref,
							u_t_ref, dt_k,
							traj_ctrl_params, mode_traj, TRAJ_PARAMS_VARIABLE_OFF, &init_traj_exerc); // NOTE: was TRAJ_PARAMS_VARIABLE_ON for both cases
					else
						// Isometric trajectory OR safety catch (TODO: merge with home_point_ellipse()?):
						traj_ref_step_isometric(
							p_ref, dt_p_ref,
							&phi_ref, &dt_phi_ref,
							u_t_ref);
				}

				// Active trajectory control:
				else if (exercise_mode == ActiveTrajectoryCtrl) {
					if (traj_exerc_type == EllipticTraj || traj_exerc_type == LinearTraj)
						traj_ref_step_active_elliptic(
							z_intern_home_dbl,
							p_ref, dt_p_ref,
							&phi_ref, &dt_phi_ref,
							u_t_ref, dt_k, F_end_in,
							traj_ctrl_params, admitt_model_params, ADMITT_MODEL_CONSTR_ON, mode_traj, TRAJ_PARAMS_VARIABLE_OFF, &init_traj_exerc); // NOTE: elliptic was TRAJ_PARAMS_VARIABLE_ON
					else
						// Isometric trajectory OR safety catch (TODO: merge with home_point_ellipse()?):
						traj_ref_step_isometric(
							p_ref, dt_p_ref,
							&phi_ref, &dt_phi_ref,
							u_t_ref);
				}

				///////////////////////////////////////////////////////////////////////////////
				// EXERCISE - SLOWING mode: detect when completed
				///////////////////////////////////////////////////////////////////////////////

				if (pace_exerc == SLOWING) {
					t_slow = t_ref - t_slow_ref;

					if (t_slow >= T_exp && fabs(dt_phi_ref) < OMEGA_THR_HOMING_START) {
						// Internal message: slowing-down completed
						msg_code_intern = SLOWING_COMPLETED_CMD;

						#if USE_ITM_OUT_RT_CHECK_CTRL
							printf("   <<test_real_time_statemach_control()>> HOMING condition detected \n\n");
							// printf("   t_slow = [%3.2f], T_exp = [%3.2f], dt_phi_ref = [%3.3f] \n\n", t_slow, T_exp, dt_phi_ref);
						#endif
					}
				}
			}

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// STATE-BASED ACTIONS (3): FINAL
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			// Actions for active system:
			if (state_fw != ST_FW_SYSTEM_OFF) {

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
				F_end_cmd_gcomp[IDX_Y] = SCALE_GCOMP[idx_scale_gain]*F_G_COMP_DEF + F_gcomp_dyn;

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
				nml_mat_smult_r(F_end_cmd_fb_nml, SCALE_FB[idx_scale_gain]); // scale by switching variable

				// FB force commands, array form:
				for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
					F_end_cmd_fb[c_i] = F_end_cmd_fb_nml->data[c_i][0];

				///////////////////////////////////////////////////////////////////////////////
				// FF force command:
				///////////////////////////////////////////////////////////////////////////////

				// FF force commands, array form:
				for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
					F_end_cmd_ff[c_i] = SCALE_FF[idx_scale_gain]*C_FF_DC_DEF[c_i]*dt_p_ref[c_i];

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
					F_end_cmd_int_err[c_i] = SCALE_INT_ERR[idx_scale_gain]*err_int_pos[c_i];

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

				// ITM Console output:
				#if USE_ITM_OUT_RT_CHECK_STATES
					if (state_fw_changed || rt_step_i % (DT_DISP_MSEC_REALTIME/DT_STEP_MSEC) == 0) {
						printf("   [%s]  exerc mode [%s]  p_ref [%3.3f, %3.3f]  ",
								STR_ST_FW[state_fw - OFFS_ST_FW],
								EXERC_MODE_STR[exercise_mode],
								p_ref[IDX_X], p_ref[IDX_Y]);

						printf("mot_trq_active [%d]  mot_alert [%d]  pace_exerc [%s]  mode_traj [%d]  t_slow [%3.2f] T_exp [%3.2f]  dt_phi_ref [%3.2f]\n\n",
								motor_torque_active, motor_alert,
								PACE_EXERC_STR[pace_exerc],
								mode_traj, t_slow, T_exp, dt_phi_ref);
					}
				#endif

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

					// ITM Console output:
					/*
					#if USE_ITM_OUT_RT_CHECK_CTRL
						printf("   <<test_real_time_statemach_control()>> MOTOR ALERT = [%d] \n\n", motor_alert);
					#endif
					*/
				}
			} // end if (state_fw != ST_FW_SYSTEM_OFF) - FINAL

			///////////////////////////////////////////////////////////////////////////////
			// Send feedback data to TCP/IP app:
			///////////////////////////////////////////////////////////////////////////////

			#if !OVERR_DATA_FEEDBACK
				if (state_fw == ST_FW_EXERCISE_ON) {
					// HACK: include low-pass filtered force sensor measurements in robot readings:
					LL_mech_readings.Xforce = F_end_lo[IDX_X];
					LL_mech_readings.Yforce = F_end_lo[IDX_Y];

					send_lowerlimb_exercise_feedback(up_time, &LL_mech_readings, &LL_motors_settings, &ref_kinematics);
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			// Track changes of state / command code:
			///////////////////////////////////////////////////////////////////////////////

			state_fw_prev   = state_fw;
			pace_exerc_prev = pace_exerc;

			///////////////////////////////////////////////////////////////////////////////
			// ITM console output:
			///////////////////////////////////////////////////////////////////////////////

			#if USE_ITM_OUT_RT_CHECK_CTRL_LONG
				if (rt_step_i % (DT_DISP_MSEC_REALTIME/DT_STEP_MSEC) == 0) {
					// Check uptime after computations:
					up_time_end = getUpTime();

					printf("   ----------------------------\n");
					printf("   %d\t%3.3f\t(%d)\tphi = [%3.2f]\tdt_phi = [%3.2f]\tp_ref = [%3.3f, %3.3f]\tdt_p_ref = [%3.3f, %3.3f]\n",
						rt_step_i,
						t_ref,
						(int)up_time_end - (int)up_time,
						phi_ref, dt_phi_ref,
						p_ref[IDX_X], p_ref[IDX_Y],
						dt_p_ref[IDX_X], dt_p_ref[IDX_Y]);

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
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			// Increase real-time step counter:
			///////////////////////////////////////////////////////////////////////////////

			rt_step_i++;

		} // end if (up_time >= algo_nextTime) // RT time step
	} // end while (t_ref <= T_RUN_MAX)
}

