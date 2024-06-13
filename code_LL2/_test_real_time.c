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
// TEST SCRIPT - REAL-TIME- GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_real_time(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3) {

	/////////////////////////////////////////////////////////////////////////////////////
	// MOTOR STATE VARS - GAO:
	/////////////////////////////////////////////////////////////////////////////////////

	uint8_t motor_result      = 0;
	uint8_t motor_alert       = 0;

	/////////////////////////////////////////////////////////////////////////////////////
	// FIRMWARE/CONTROL PARAMETERS - GAO:
	/////////////////////////////////////////////////////////////////////////////////////

	uint64_t up_time;
	uint64_t up_time_end;

	lowerlimb_mech_readings_t   LL_mech_readings;
	lowerlimb_motors_settings_t LL_motors_settings;
	traj_ctrl_params_t          traj_ctrl_params;
	admitt_model_params_t       admitt_model_params;
	lowerlimb_ref_kinematics_t	ref_kinematics;

	const int IS_CALIBRATION = 1; // what did this flag do in update_motor_algo() (now set_LL_mech_readings())?

	/////////////////////////////////////////////////////////////////////////////////////
	// Sensor variables:
	/////////////////////////////////////////////////////////////////////////////////////

	// Force sensor readings, raw:
	uint32_t force_end_in_x_sensor = 0;
	uint32_t force_end_in_y_sensor = 0;

	// End-effector force measurements:
	double F_end_m[N_COORD_2D];

	/////////////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - REFERENCE:
	/////////////////////////////////////////////////////////////////////////////////////

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
	double z_intern_o_dbl[2*N_COORD_EXT];

	/////////////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - MEASURED:
	/////////////////////////////////////////////////////////////////////////////////////

	// Measured position and velocity:
	double    p_m[N_COORD_2D];
	double dt_p_m[N_COORD_2D];

	/////////////////////////////////////////////////////////////////////////////////////
	// Force command:
	/////////////////////////////////////////////////////////////////////////////////////

	float force_end_cmd[N_COORD_2D] = {0.0, 0.0};

	/////////////////////////////////////////////////////////////////////////////////////
	// TCP/IP variables:
	/////////////////////////////////////////////////////////////////////////////////////

	int sock_status;

	/////////////////////////////////////////////////////////////////////////////////////
	// App variables:
	/////////////////////////////////////////////////////////////////////////////////////

	uint16_t cmd_code  = 0;
	uint8_t  app_state = 0;

	uint8_t system_state_prev   = LL_sys_info.system_state;
	uint8_t activity_state_prev = LL_sys_info.activity_state;
	uint8_t exercise_state_prev = LL_sys_info.exercise_state;

	bool brake_cmd;

	uint8_t ADMITT_CTRL_MODE;

	/////////////////////////////////////////////////////////////////////////////////////
	// Counters and timers:
	/////////////////////////////////////////////////////////////////////////////////////

	// Real-time counters and timers:
	double T_RUN_MAX    = 1000;
	double t_ref        = 0.0;

	int rt_step_i       = 0; // real-time step counter
	int c_i             = 0; // general-purpose counter
	int8_t  switch_traj = 0;

	// TCP communication checks:
	int32_t ret_tcp_msg      = 0;
	uint64_t dt_ret_tcp_msec = 0;

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize app:
	/////////////////////////////////////////////////////////////////////////////////////

	app_state = lowerlimb_app_state_initialize(0, VER_H, VER_L, VER_P, &LL_motors_settings);

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize motor algorithm:
	/////////////////////////////////////////////////////////////////////////////////////

	init_motor_algo(&LL_mech_readings, &LL_motors_settings);

	/////////////////////////////////////////////////////////////////////////////////////
	// Initial conditions for kinematics:
	/////////////////////////////////////////////////////////////////////////////////////

	// Positions:
	z_intern_o_dbl[0] = 0;
	z_intern_o_dbl[1] = 0;
	z_intern_o_dbl[2] = 0;

	// Velocities:
	z_intern_o_dbl[3] = 0;
	z_intern_o_dbl[4] = 0;
	z_intern_o_dbl[5] = 0;

	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////
	// USER CODE LOOP - GAO
	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////

	while (t_ref <= T_RUN_MAX) {

		/////////////////////////////////////////////////////////////////////////////////////
		// Get uptime in milliseconds:
		/////////////////////////////////////////////////////////////////////////////////////

		up_time = getUpTime();

		/////////////////////////////////////////////////////////////////////////////////////
		// Execute real-time step:
		/////////////////////////////////////////////////////////////////////////////////////

		if (up_time >= algo_nextTime) { // RT time step

			/////////////////////////////////////////////////////////////////////////////////////
			// Set up next RT time point (msec):
			/////////////////////////////////////////////////////////////////////////////////////

			algo_nextTime = up_time + DT_STEP_MSEC;

			/////////////////////////////////////////////////////////////////////////////////////
			// Update reference time:
			/////////////////////////////////////////////////////////////////////////////////////

			t_ref = dt_k*rt_step_i;

			/////////////////////////////////////////////////////////////////////////////////////
			// uart rx state check:
			/////////////////////////////////////////////////////////////////////////////////////

			// uart_rx_data_state();

			/////////////////////////////////////////////////////////////////////////////////////
			// Ethernet connection:
			/////////////////////////////////////////////////////////////////////////////////////

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

			////////////////////////////////////////////////////////////////////////////////////////
			// GET TCP/IP APP STATE - GAO
			////////////////////////////////////////////////////////////////////////////////////////

			#if USE_APP_STATE_TEMPLATE
				// Template function for the firmware state machine:
				LL_sys_info = lowerlimb_app_state_template(Read_Haptic_Button(), motor_alert,
						&traj_ctrl_params, &admitt_model_params, &LL_motors_settings, &cmd_code);
			#else
				LL_sys_info = lowerlimb_app_state(Read_Haptic_Button(), motor_alert,
						&traj_ctrl_params, &admitt_model_params, &LL_motors_settings, &cmd_code);
			#endif

			// HACK: override setup state
			if (LL_sys_info.exercise_state == SETUP)
				LL_sys_info.exercise_state = RUNNING;

			/////////////////////////////////////////////////////////////////////////////////////
			// Clear motor_alert after sending it to TCP/IP APP state:
			/////////////////////////////////////////////////////////////////////////////////////

			motor_alert = 0;

			////////////////////////////////////////////////////////////////////////////////////////
			// Override trajectory and control parameters - TODO: remove at a later date
			////////////////////////////////////////////////////////////////////////////////////////

			#if OVERR_DYN_PARAMS_RT
				traj_ctrl_params.cycle_period   = 3.0;
				traj_ctrl_params.exp_blend_time = 10.0;
				traj_ctrl_params.semiaxis_x     = 0.15;
				traj_ctrl_params.semiaxis_y     = 0.08;
				traj_ctrl_params.rot_angle      = 0;
				traj_ctrl_params.cycle_dir      = 1;

				static double damp_ratio = 0.2;
				static double w_n        = 2*PI*1.0; // natural frequency
				static double sigma;

				admitt_model_params.inertia_x = admitt_model_params.inertia_y = 10.0;

				if (ADMITT_MODEL_CONSTR_ON) {
					admitt_model_params.stiffness =  0.0;
					admitt_model_params.damping   = 25.0;
				}
				else {
					admitt_model_params.stiffness = admitt_model_params.inertia_x*w_n*w_n;
					admitt_model_params.damping   = 2*damp_ratio*sqrt(admitt_model_params.stiffness*admitt_model_params.inertia_x);
				}

				sigma = damp_ratio*w_n;
				// T_LOOP = 6.0/sigma;

				admitt_model_params.p_eq_x    = 0;
				admitt_model_params.p_eq_y    = 0;
				admitt_model_params.Fx_offset = 0;
				admitt_model_params.Fy_offset = 0;
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

			/////////////////////////////////////////////////////////////////////////////////////
			// if system is on or off?
			/////////////////////////////////////////////////////////////////////////////////////

			if (LL_sys_info.system_state == ON) {

				/////////////////////////////////////////////////////////////////////////////////////
				// Update LEDs state:
				/////////////////////////////////////////////////////////////////////////////////////

				cycle_haptic_buttons();

				/////////////////////////////////////////////////////////////////////////////////////
				// Check emergency signal GPIOG GPIO_PIN_14:
				/////////////////////////////////////////////////////////////////////////////////////

				/////////////////////////////////////////////////////////////////////////////////////
				// Check brakes:
				/////////////////////////////////////////////////////////////////////////////////////

				// set_brakes_timed(up_time, &brakes_nextTime);

				/////////////////////////////////////////////////////////////////////////////////////
				// Update safety:
				/////////////////////////////////////////////////////////////////////////////////////

				set_safetyOff(LL_sys_info.safetyOFF);

				/////////////////////////////////////////////////////////////////////////////////////
				// Check exercise start / stop switches:
				/////////////////////////////////////////////////////////////////////////////////////

				if (exercise_state_prev != RUNNING && LL_sys_info.exercise_state == RUNNING)
					switch_traj = SWITCH_TRAJ_START;
				else if ((exercise_state_prev != SLOWING && LL_sys_info.exercise_state == SLOWING) ||
						 (exercise_state_prev != STOPPED && LL_sys_info.exercise_state == STOPPED))
					switch_traj = SWITCH_TRAJ_END;
				else
					switch_traj = SWITCH_TRAJ_NULL;

				/////////////////////////////////////////////////////////////////////////////////////
				/////////////////////////////////////////////////////////////////////////////////////
				// SWITCH ACTIVITY STATE - GAO
				/////////////////////////////////////////////////////////////////////////////////////
				/////////////////////////////////////////////////////////////////////////////////////

				if (LL_sys_info.activity_state == IDLE) {

					/////////////////////////////////////////////////////////////////////////////////////
					// IDLE state:
					/////////////////////////////////////////////////////////////////////////////////////

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

				else if (LL_sys_info.activity_state == CALIB) {

					/////////////////////////////////////////////////////////////////////////////////////
					// CALIBRATION state:
					/////////////////////////////////////////////////////////////////////////////////////

				} // end if (LL_sys_info.activity_state == CALIB)

				else if (LL_sys_info.activity_state == EXERCISE) {

					/////////////////////////////////////////////////////////////////////////////////////
					// EXERCISE state:
					/////////////////////////////////////////////////////////////////////////////////////

					if (LL_sys_info.exercise_state == RUNNING || LL_sys_info.exercise_state == SLOWING) {

						/////////////////////////////////////////////////////////////////////////////////////
						// Retrieve force sensor readings:
						/////////////////////////////////////////////////////////////////////////////////////

						force_sensors_read(hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor, &dum_force_end_in_x, &dum_force_end_in_y);
						current_sensors_read(hadc1, &current_sensor_L, &current_sensor_R);

						/////////////////////////////////////////////////////////////////////////////////////
						// Get lower-limb robot sensor readings:
						/////////////////////////////////////////////////////////////////////////////////////

						motor_alert = set_LL_mech_readings(&LL_mech_readings, up_time,
							qei_count_L_read(), qei_count_R_read(),
							current_sensor_L, current_sensor_R,
							force_end_in_x_sensor, force_end_in_y_sensor,
							IS_CALIBRATION);

						#if OVERR_FORCE_SENSORS_CALIB
							LL_mech_readings.Xforce = 0; // 0.1*cos(2*PI*t_ref + PI/2);
							LL_mech_readings.Yforce = 0; // 0.1*cos(2*PI*t_ref);
						#endif

						/////////////////////////////////////////////////////////////////////////////////////
						// Extract measured end-effector forces:
						/////////////////////////////////////////////////////////////////////////////////////

						F_end_m[IDX_X] = (double)LL_mech_readings.Xforce;
						F_end_m[IDX_Y] = (double)LL_mech_readings.Yforce;

						/////////////////////////////////////////////////////////////////////////////////////
						// Extract measured position and velocity from sensor readings:
						/////////////////////////////////////////////////////////////////////////////////////

						p_m[IDX_X]     = (double)LL_mech_readings.coord.x;
						p_m[IDX_Y]     = (double)LL_mech_readings.coord.y;

						dt_p_m[IDX_X]  = (double)LL_mech_readings.velocity.x;
						dt_p_m[IDX_Y]  = (double)LL_mech_readings.velocity.y;

						/////////////////////////////////////////////////////////////////////////////////////
						// Generate reference trajectory:
						/////////////////////////////////////////////////////////////////////////////////////

						// Generate trajectory points:
						if (LL_sys_info.exercise_mode == PassiveTrajectoryCtrl)
							traj_ref_step_passive_elliptic(
								p_ref, dt_p_ref,
								&phi_ref, &dt_phi_ref,
								u_t_ref, dt_k,
								traj_ctrl_params, switch_traj, USE_TRAJ_PARAMS_VARIABLE);
						/*
						else if (LL_sys_info.exercise_mode == AdmittanceCtrl)
							p_ref, dt_p_ref,
							&phi_ref, &dt_phi_ref,
							u_t_ref, dt_k, F_end_m, z_intern_o_dbl,
							traj_ctrl_params, admitt_model_params, ADMITT_MODEL_CONSTR_OFF, switch_traj);
						*/
						else if (LL_sys_info.exercise_mode == ActiveTrajectoryCtrl)
							traj_ref_step_active_elliptic(
								p_ref, dt_p_ref,
								&phi_ref, &dt_phi_ref,
								u_t_ref, dt_k, F_end_m, z_intern_o_dbl,
								traj_ctrl_params, admitt_model_params, ADMITT_MODEL_CONSTR_ON, switch_traj, USE_TRAJ_PARAMS_VARIABLE);
						else
							printf("   exercise_mode: [%s]\n", EXERC_MODE_STR[LL_sys_info.exercise_mode]);

						// Set reference kinematics struct:
						for (int c_i = 0; c_i < N_COORD_2D; c_i++) {
							ref_kinematics.p_ref[c_i]    = p_ref[c_i];
							ref_kinematics.dt_p_ref[c_i] = dt_p_ref[c_i];
						}

						ref_kinematics.phi_ref    = phi_ref;
						ref_kinematics.dt_phi_ref = dt_phi_ref;

						/////////////////////////////////////////////////////////////////////////////////////
						// Compute force command in end-effector coordinates:
						/////////////////////////////////////////////////////////////////////////////////////

						/*
						// MATLAB control code (TODO: delete at a later date):

						// Feedback gains (xv = [x dt_x y dt_y]):
						K_lq_xv_d =
						   10.2006    0.6046         0         0
						         0         0   10.2007    0.0839

						// FF 'transfer function' (dc_C_ff):
						749.2168
						794.0586

						// Gravity compensation force (F_g_comp):
						15

						// FB force command:
						F_end_cmd_lq = sw_lq*K_lq_xv_d*(xv_ref - xv_m)

						// FF force command:
						F_end_cmd_ff = sw_ff*dc_C_ff.*dt_p_ref(step_i, :).'

						// Gravity compensation command:
						F_end_cmd_g  = sw_g*[0; F_g_comp]

						// End-effector force command:
						F_end_cmd(step_i, :) = (F_end_cmd_lq + F_end_cmd_ff + F_end_cmd_g).'
						*/

						/////////////////////////////////////////////////////////////////////////////////////
						// UPDATE MOTOR SETTINGS - GAO:
						/////////////////////////////////////////////////////////////////////////////////////

						// Clear motors settings:
						clear_lowerlimb_motors_settings(&LL_motors_settings);

						set_LL_motor_settings(&LL_motors_settings, force_end_cmd);

						/////////////////////////////////////////////////////////////////////////////////////
						// Send motor commands:
						/////////////////////////////////////////////////////////////////////////////////////

						#if MOTOR_TORQUE_ON
							// Check if need to end exercise:
							if (motor_alert == 1 || motor_alert == 2) {
								stop_exercise(&LL_motors_settings);

								// Disable motors:
								motor_L_move(0, false, false);
								motor_L_move(0, false, false);
							}
							else {
								motor_L_move(LL_motors_settings.right.dac_in, LL_motors_settings.right.motor_direction,
									LL_motors_settings.right.en_motor_driver);
								motor_L_move(LL_motors_settings.left.dac_in, LL_motors_settings.left.motor_direction,
									LL_motors_settings.left.en_motor_driver);
							}
						}
						#endif

						/////////////////////////////////////////////////////////////////////////////////////
						// Distal Force Sensor - Change only when updating TCP Protocol
						// Input Brakes info from TCP System Info
						/////////////////////////////////////////////////////////////////////////////////////

						send_lowerlimb_exercise_feedback(getUpTime(), &LL_mech_readings, &LL_motors_settings, &ref_kinematics); //  was up_time

						/////////////////////////////////////////////////////////////////////////////////////
						// ITM console output:
						/////////////////////////////////////////////////////////////////////////////////////

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
					} // end if (LL_sys_info.exercise_state == RUNNING)

					else { // LL_sys_info.exercise_state: all other values
						// reset
						clear_lowerlimb_mech_readings(&LL_mech_readings);
						clear_lowerlimb_motors_settings(&LL_motors_settings);
						clear_ctrl_params();

						// Set the motors to 0 and disable the motor driver
						motor_L_move(0, false, false);
						motor_L_move(0, false, false);
					} // end LL_sys_info.exercise_state cases
				} // end if (LL_sys_info.activity_state == EXERCISE)
			} // end if (LL_sys_info.system_state == ON)

			else {
				LED_sys_state_off();

				l_brakes(ENGAGE_BRAKES);
				r_brakes(ENGAGE_BRAKES);
			} // end LL_sys_info.system_state != ON

			/////////////////////////////////////////////////////////////////////////////////////
			// Check if need to dump UART FIFO
			/////////////////////////////////////////////////////////////////////////////////////

			/*
			if (prev_fifo_size != rx_fifo_size()) {
				prev_fifo_size = rx_fifo_size();
				expire_nextTime = up_time + DT_EXPIRE_MSEC;
			}

			if ((up_time >= expire_nextTime) && (rx_fifo_size() > 0)) {
				rx_fifo_clear();
				prev_fifo_size = 0;
			}
			*/

			/////////////////////////////////////////////////////////////////////////////////////
			// Track changes of state:
			/////////////////////////////////////////////////////////////////////////////////////

			system_state_prev   = LL_sys_info.system_state;
			activity_state_prev = LL_sys_info.activity_state;
			exercise_state_prev = LL_sys_info.exercise_state;

			/////////////////////////////////////////////////////////////////////////////////////
			// ITM console output:
			/////////////////////////////////////////////////////////////////////////////////////

			#if USE_ITM_OUT_RT_CHECK
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
				}
			#endif

			/////////////////////////////////////////////////////////////////////////////////////
			// Increase real-time step counter:
			/////////////////////////////////////////////////////////////////////////////////////

			rt_step_i++;

		} // end if (up_time >= algo_nextTime) // RT time step
	} // end while (t_ref <= T_RUN_MAX)
}
