/////////////////////////////////////////////////////////////////////////////
//
//  _test_real_time_stmachine_tcp_app.c
//
// Created on: 2024.07.04
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <_test_real_time.h>

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// TEST SCRIPT - REAL-TIME
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_real_time_stmachine_tcp_app(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3) {

	///////////////////////////////////////////////////////////////////////////////
	// MOTOR STATE VARS:
	///////////////////////////////////////////////////////////////////////////////

	uint8_t motor_alert         = 0;
	// uint8_t motor_result        = 0;

	///////////////////////////////////////////////////////////////////////////////
	// FIRMWARE / CONTROL PARAMETERS:
	///////////////////////////////////////////////////////////////////////////////

	// Select message type to receive (see lowerlimb_app.h):
	uint8_t USE_VARS_TCP_MSG = 0;

	uint64_t up_time;
	uint64_t up_time_end;

	lowerlimb_mech_readings_t   LL_mech_readings;
	lowerlimb_motors_settings_t LL_motors_settings;
	traj_ctrl_params_t          traj_ctrl_params;
	admitt_model_params_t       admitt_model_params;
	// lowerlimb_ref_kinematics_t	ref_kinematics;

	// const int is_calibration = 1; // what did this flag do in update_motor_algo() (now set_LL_mech_readings())?

	//////////////////////////////////////////////////////////////////////////////////
	// ADMITTANCE & TRAJECTORY PARAMETERS:
	//////////////////////////////////////////////////////////////////////////////////

	#if OVERR_DYN_PARAMS_RT
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Select trajectory type:
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - REFERENCE:
	///////////////////////////////////////////////////////////////////////////////

    // Integrator time step:
	double dt_k = (double)DT_STEP_MSEC/MSEC_PER_SEC; // integrator step (initial)

	// Reference position and velocity:


	///////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - MEASURED:
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Force sensor variables:
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Force sensor variables - dynamic gravity compensation:
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Gain and scaling variables:
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Force commands:
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Additional control variables:
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	// Calibration variables:
	///////////////////////////////////////////////////////////////////////////////

	// Calibration states:
	uint8_t calib_fsens_on    = 0;
	uint8_t calib_enc_on      = 0;

	// Homing states:
	uint8_t homing_on         = 0;

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

	uint16_t cmd_code              = 0;
	uint16_t cmd_code_prev_to_last = 0;

	// uint8_t  app_state          = 0;
	// uint8_t system_state_prev   = LL_sys_info.system_state;
	// uint8_t activity_state_prev = LL_sys_info.activity_state;
	// uint8_t exercise_state_prev = LL_sys_info.exercise_state;

	///////////////////////////////////////////////////////////////////////////////
	// Real-time counters, timers and switches:
	///////////////////////////////////////////////////////////////////////////////

	double T_RUN_MAX    = 5000;
	double t_ref        = 0.0;

	int rt_step_i       = 0; // real-time step counter
	int r_i, c_i, v_i; // general-purpose counters

	int8_t switch_traj  = SWITCH_TRAJ_NULL;

	///////////////////////////////////////////////////////////////////////////////
	// Display variables:
	///////////////////////////////////////////////////////////////////////////////

	/*
	#if USE_ITM_OUT_RT_CHECK
		uint8_t idx_sys_state   = LL_sys_info.system_state;
		uint8_t idx_activ_state = LL_sys_info.activity_state;
		uint8_t idx_exerc_state = LL_sys_info.exercise_state;
	#endif
	*/

	///////////////////////////////////////////////////////////////////////////////
	// Initialize app:
	///////////////////////////////////////////////////////////////////////////////

	lowerlimb_app_state_initialize(0, VER_H, VER_L, VER_P, &LL_motors_settings); // was app_state =

	///////////////////////////////////////////////////////////////////////////////
	// Initialize motor algorithm:
	///////////////////////////////////////////////////////////////////////////////

	init_motor_algo(&LL_mech_readings, &LL_motors_settings);

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
			// Receive command message from TCP/IP app:
			//////////////////////////////////////////////////////////////////////////////////

			is_valid_cmd_code_tcp(&cmd_code, Read_Haptic_Button(), motor_alert, USE_VARS_TCP_MSG);

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// STATE MACHINE:
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			// STATE MACHINE end

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// HARDWARE actions listing:
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Clear motor_alert after sending it to TCP/IP APP state:
			///////////////////////////////////////////////////////////////////////////////

			motor_alert = 0; // TODO: what does this do?

			///////////////////////////////////////////////////////////////////////////////
			// Update LEDs state:
			///////////////////////////////////////////////////////////////////////////////

			// cycle_haptic_buttons(); // TODO: check logic
			// LED_sys_state_off(); // TODO: where should this go?

			///////////////////////////////////////////////////////////////////////////////
			// Update safety:
			///////////////////////////////////////////////////////////////////////////////

			set_safetyOff(LL_sys_info.safetyOFF); // TODO: what does this do?

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// SENSOR readings:
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Retrieve force sensor readings:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Get lower-limb robot sensor readings:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Extract measured position and velocity from sensor readings:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Extract measured end-effector forces:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Scale array selector (CRITICAL):
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// Set reference kinematics struct:
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////
			// End-effector force commands:
			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Gravity compensation force command (end-effector coordinates):
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// FB force command:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// FF force command:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Integral position error force command:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Total force command:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Update motors' settings:
			///////////////////////////////////////////////////////////////////////////////

			// clear_lowerlimb_motors_settings(&LL_motors_settings);
			// set_LL_motor_settings(&LL_motors_settings, F_end_cmd);

			///////////////////////////////////////////////////////////////////////////////
			// Send motor commands:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Send feedback data to TCP/IP app:
			///////////////////////////////////////////////////////////////////////////////

			///////////////////////////////////////////////////////////////////////////////
			// Track changes of state:
			///////////////////////////////////////////////////////////////////////////////

			/*
			system_state_prev   = LL_sys_info.system_state;
			activity_state_prev = LL_sys_info.activity_state;
			exercise_state_prev = LL_sys_info.exercise_state;
			*/

			cmd_code_prev_to_last = cmd_code;

			///////////////////////////////////////////////////////////////////////////////
			// ITM console output:
			///////////////////////////////////////////////////////////////////////////////

			#if USE_ITM_OUT_RT_CHECK_LONG
				if (rt_step_i % (DT_DISP_MSEC_REALTIME/DT_STEP_MSEC) == 0) {

					// Check uptime after computations:
					up_time_end = getUpTime();

					if ((up_time_end - up_time) > DT_STEP_MSEC) {
						printf("\n");
						printf("   rt_step_i = [%d], rt_step_i = [%3.3f], D_up_time = [%d] \n",
							rt_step_i,
							t_ref,
							(int)up_time_end - (int)up_time);

						printf("   cmd_code_prev_to_last = [%s], cmd_code = [%s] \n",
									MSG_TCP_STR[cmd_code_prev_to_last], MSG_TCP_STR[cmd_code]);
						printf("   LL_sys_info.activity_state = [%s] \n\n",   ACTIV_STATE_STR[LL_sys_info.activity_state]);
					}
				}
			#endif

			///////////////////////////////////////////////////////////////////////////////
			// Increase real-time step counter:
			///////////////////////////////////////////////////////////////////////////////

			rt_step_i++;

		} // end if (up_time >= algo_nextTime) // RT time step
	} // end while (t_ref <= T_RUN_MAX)
}
