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

	const int MOTOR_TORQUE_ON = 0;

	uint8_t motor_result      = 0;
	uint8_t motor_alert       = 0;
	uint8_t prevConnected     = 0;

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
	// Counters and timers:
	/////////////////////////////////////////////////////////////////////////////////////

	double T_RUN_MAX;

	int step_i = 0;
	double t_ref = 0;

	/////////////////////////////////////////////////////////////////////////////////////
	// Force command:
	/////////////////////////////////////////////////////////////////////////////////////

	float force_end_cmd[N_COORD_2D] = {0.0, 0.0};

	/////////////////////////////////////////////////////////////////////////////////////
	// TCP app:
	/////////////////////////////////////////////////////////////////////////////////////

	lowerlimb_tcp_app_state_initialize(0, VER_H, VER_L, VER_P, &LL_motors_settings);

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize motor algorithm:
	/////////////////////////////////////////////////////////////////////////////////////

	init_motor_algo(&LL_mech_readings, &LL_motors_settings);

	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////
	// USER CODE BEGIN WHILE - GAO
	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////

	do {

		/////////////////////////////////////////////////////////////////////////////////////
		// uart rx state check
		/////////////////////////////////////////////////////////////////////////////////////

		uart_rx_data_state();

		/////////////////////////////////////////////////////////////////////////////////////
		// ethernet check
		/////////////////////////////////////////////////////////////////////////////////////

		ethernet_w5500_state();

		////////////////////////////////////////////////////////////////////////////////////////
		// GET TCP/IP APP STATE - GAO
		////////////////////////////////////////////////////////////////////////////////////////

		LL_sys_info = lowerlimb_tcp_app_state(Read_Haptic_Button(), motor_alert,
				&traj_ctrl_params, &admitt_model_params, &LL_motors_settings);

		/////////////////////////////////////////////////////////////////////////////////////
		// Clear motor_alert after sending into tcp app state:
		/////////////////////////////////////////////////////////////////////////////////////

		motor_alert = 0;

		////////////////////////////////////////////////////////////////////////////////////////
		// Override trajectory and control parameters - TODO: remove at a later date
		////////////////////////////////////////////////////////////////////////////////////////

#if OVERR_DYN_PARAMS_RT
		/*
		traj_ctrl_params.cycle_period = 3.0;
		traj_ctrl_params.exp_blend_time = 3.0;
		traj_ctrl_params.semiaxis_x = 0.15;
		traj_ctrl_params.semiaxis_y = 0.08;
		traj_ctrl_params.rot_angle = 0;
		// traj_ctrl_params.cycle_dir = 1;
		*/

		static double damp_ratio = 0.2;
		static double w_n        = 2*PI*1.0; // natural frequency
		static double sigma;

		admitt_model_params.inertia_x = admitt_model_params.inertia_y = 10.0;
		admitt_model_params.stiffness = admitt_model_params.inertia_x*w_n*w_n;

		admitt_model_params.damping =
				2*damp_ratio*
				sqrt(admitt_model_params.stiffness*admitt_model_params.inertia_x);

		sigma = damp_ratio*w_n;
		T_RUN_MAX = 6.0/sigma;

		admitt_model_params.p_eq_x    = 0;
		admitt_model_params.p_eq_y    = 0;
		admitt_model_params.Fx_offset = 0;
		admitt_model_params.Fy_offset = 0;
#endif

#if USE_ITM_OUT_GUI_PARAMS
		if (step_i > 0 && step_i % (DT_DISP_MSEC_GUI_PARAMS/(int)(1000*dt_k)) == 0) {
			printf("___________________________________\n");
			printf("step_i   = [%d]\n", step_i);
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
		// Execute real-time step:
		/////////////////////////////////////////////////////////////////////////////////////

		up_time = getUpTime();

		if (up_time >= algo_nextTime) {

			algo_nextTime = up_time + DT_STEP_MSEC;

			/////////////////////////////////////////////////////////////////////////////////////
			// if system is on or off?
			/////////////////////////////////////////////////////////////////////////////////////

			if (LL_sys_info.system_state == ON) {

				/////////////////////////////////////////////////////////////////////////////////////
				// Check emergency signal GPIOG GPIO_PIN_14
				/////////////////////////////////////////////////////////////////////////////////////

				cycle_haptic_buttons();
				set_brakes_timed(up_time, &brakes_nextTime);

				/////////////////////////////////////////////////////////////////////////////////////
				// update safety
				/////////////////////////////////////////////////////////////////////////////////////

				set_safetyOff(LL_sys_info.safetyOFF);

				/////////////////////////////////////////////////////////////////////////////////////
				/////////////////////////////////////////////////////////////////////////////////////
				// SWITCH ACTIVITY STATE - GAO
				/////////////////////////////////////////////////////////////////////////////////////
				/////////////////////////////////////////////////////////////////////////////////////

				switch (LL_sys_info.activity_state) {

					/////////////////////////////////////////////////////////////////////////////////////
					// IDLE state:
					/////////////////////////////////////////////////////////////////////////////////////

					case IDLE:
						// clear motor algo readings and settings
						clear_lowerlimb_mech_readings(&LL_mech_readings);
						clear_lowerlimb_motors_settings(&LL_motors_settings);
						clear_ctrl_params();

						// disable motor
						motor_R_move(0, false, false);
						motor_L_move(0, false, false);

						// restart
						init_motor_algo(&LL_mech_readings, &LL_motors_settings);

						// Reset calibration state:
						// reset_calibration_state();

						break;

					/////////////////////////////////////////////////////////////////////////////////////
					// CALIBRATION state:
					/////////////////////////////////////////////////////////////////////////////////////

					case CALIB: // To-do (REMOVED)
						break;

					/////////////////////////////////////////////////////////////////////////////////////
					// EXERCISE state:
					/////////////////////////////////////////////////////////////////////////////////////

					case EXERCISE:
							if (LL_sys_info.exercise_state == RUNNING) {

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
#if OVERR_FORCE_SENSORS_CALIB_1
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
								// Motor algorithm computation:
								/////////////////////////////////////////////////////////////////////////////////////

								traj_ref_step_active_elliptic(
										p_ref, dt_p_ref,
										&phi_ref, &dt_phi_ref,
										u_t_ref, dt_k, F_end_m, z_intern_o_dbl,
										traj_ctrl_params, admitt_model_params, USE_ADMITT_MODEL_CONSTR_RT);

								// Set reference kinematics struct:
								for (int c_i = 0; c_i < N_COORD_2D; c_i++) {
									ref_kinematics.p_ref[c_i]    = p_ref[c_i];
									ref_kinematics.dt_p_ref[c_i] = dt_p_ref[c_i];
								}

								ref_kinematics.phi_ref    = phi_ref;
								ref_kinematics.dt_phi_ref = dt_phi_ref;

								/////////////////////////////////////////////////////////////////////////////////////
								// UPDATE MOTOR SETTINGS - GAO:
								/////////////////////////////////////////////////////////////////////////////////////

								// Clear motors settings:
							    clear_lowerlimb_motors_settings(&LL_motors_settings);

								set_LL_motor_settings(&LL_motors_settings, force_end_cmd);

								/////////////////////////////////////////////////////////////////////////////////////
								// Send motor commands:
								/////////////////////////////////////////////////////////////////////////////////////

								if (MOTOR_TORQUE_ON) {
									// Check if need to end exercise:
									if ((motor_alert == 1) || (motor_alert == 2)) {
										stop_exercise(&LL_motors_settings);

										// Disable motors:
										motor_R_move(0, false, false);
										motor_L_move(0, false, false);
									}
									else {
										motor_R_move(LL_motors_settings.right.dac_in, LL_motors_settings.right.motor_direction,
											LL_motors_settings.right.en_motor_driver);
										motor_L_move(LL_motors_settings.left.dac_in, LL_motors_settings.left.motor_direction,
											LL_motors_settings.left.en_motor_driver);
									}
								}

								/////////////////////////////////////////////////////////////////////////////////////
								// Distal Force Sensor - Change only when updating TCP Protocol
								// Input Brakes info from TCP System Info
								/////////////////////////////////////////////////////////////////////////////////////

								set_lowerlimb_exercise_feedback(up_time, &LL_mech_readings, &LL_motors_settings, &ref_kinematics);

								/////////////////////////////////////////////////////////////////////////////////////
								// Update step time:
								/////////////////////////////////////////////////////////////////////////////////////

								t_ref = dt_k*step_i;

								// Check uptime after computations:
								up_time_end = getUpTime();

								// ITM console output:
#if USE_ITM_OUT_REAL_TIME
								if ((up_time_end - up_time) > DT_STEP_MSEC) {

									printf("____________________________\n");
									printf("step_i [%d]: t_ref = %f\tDT MSEC = %d\n",
										step_i,
										t_ref,
										(int)(up_time_end - up_time));
									printf("____________________________\n");
								}

								/*
								else if (step_i % (DT_DISP_MSEC_REALTIME/(int)(1000*dt_k)) == 0)
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
#endif
								step_i++;
							} // end if (LL_sys_info.exercise_state == ) (case: RUNNING)
							else {
								// reset
								clear_lowerlimb_mech_readings(&LL_mech_readings);
								clear_lowerlimb_motors_settings(&LL_motors_settings);
								clear_ctrl_params();

								// Set the motors to 0 and disable the motor driver
								motor_R_move(0, false, false);
								motor_L_move(0, false, false);

							} // end if (LL_sys_info.exercise_state == RUNNING) (case: !RUNNING)
					} // switch (LL_sys_info.activity_state)
			} // if (LL_sys_info.system_state == ON)
			else {
				LED_sys_state_off();
			}

			/////////////////////////////////////////////////////////////////////////////////////
			// indicate system was TCP connected
			/////////////////////////////////////////////////////////////////////////////////////

			prevConnected = 1;

			/////////////////////////////////////////////////////////////////////////////////////
			// Check if need to dump UART FIFO
			/////////////////////////////////////////////////////////////////////////////////////

			if (prev_fifo_size != rx_fifo_size()) {
				prev_fifo_size = rx_fifo_size();
				expire_nextTime = up_time + DT_EXPIRE_MSEC;
			}

			if ((up_time >= expire_nextTime) && (rx_fifo_size() > 0)) {
				rx_fifo_clear();
				prev_fifo_size = 0;
			}
		} // if (up_time >= algo_nextTime)
	} while (t_ref <= T_RUN_MAX);
}
