/////////////////////////////////////////////////////////////////////////////
//
//  _test_real_time_onepass_software.c
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
test_real_time_onepass_software(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3) {

	///////////////////////////////////////////////////////////////////////////////
	// General settings:
	///////////////////////////////////////////////////////////////////////////////

	const uint8_t USE_SOFTWARE_MSG_LIST = 1; // CRITICAL option (2)

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
		// RESTORE
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Select trajectory type:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

	///////////////////////////////////////////////////////////////////////////////
	// Real-time time step, sec:
	///////////////////////////////////////////////////////////////////////////////

	double dt_k = (double)DT_STEP_MSEC/MSEC_PER_SEC;

	///////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - REFERENCE:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

	///////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - MEASURED:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

	///////////////////////////////////////////////////////////////////////////////
	// Force sensor variables:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

	///////////////////////////////////////////////////////////////////////////////
	// Force sensor variables - dynamic gravity compensation:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

	///////////////////////////////////////////////////////////////////////////////
	// Gain and scaling variables:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

	///////////////////////////////////////////////////////////////////////////////
	// Force commands:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

	///////////////////////////////////////////////////////////////////////////////
	// Additional control variables:
	///////////////////////////////////////////////////////////////////////////////

	// RESTORE

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
	uint8_t init_home_traj = 1;

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

	int8_t switch_traj = MODE_TRAJ_NULL;

	///////////////////////////////////////////////////////////////////////////////
	// Display variables:
	///////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_OUT_RT_CHECK_CTRL
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

	// RESTORE

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

			// CRITICAL: passes reference to global-declared lowerlimb_sys_info for the sake of traceability (2):
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

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Get lower-limb robot sensor readings:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Extract measured position and velocity from sensor readings:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Extract measured end-effector forces:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// EXERCISE start / stop switches:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// ACTIVITY STATE SWITCH:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				// Scale array selector (CRITICAL):
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// IDLE activity state:
				///////////////////////////////////////////////////////////////////////////////

				if (lowerlimb_sys_info.activity_state == IDLE) {

					// RESTORE
				}

				///////////////////////////////////////////////////////////////////////////////
				// CALIBRATION activity state:
				///////////////////////////////////////////////////////////////////////////////

				else if (lowerlimb_sys_info.activity_state == CALIB) { // NOTE: calib_enc_on condition is activated by lowerlimb_app_onepass_ref()

					// RESTORE

					// HACK: force response back to software:
					calib_enc_on = 0;
					send_OK_resp(Calibrate_Robot_MSG_TCP);

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
					// Exercise mode switch:
					///////////////////////////////////////////////////////////////////////////////

					if (lowerlimb_sys_info.exercise_state == RUNNING || lowerlimb_sys_info.exercise_state == SLOWING) {

						///////////////////////////////////////////////////////////////////////////////
						// SLOWING mode: reference time
						///////////////////////////////////////////////////////////////////////////////

						if (lowerlimb_sys_info.exercise_state == SLOWING && exercise_state_prev != SLOWING) {
							// RESTORE
						}

						///////////////////////////////////////////////////////////////////////////////
						// Generate reference trajectory:
						///////////////////////////////////////////////////////////////////////////////

						// RESTORE

						///////////////////////////////////////////////////////////////////////////////
						// SLOWING exercise mode: detect "ready for HOMING" activity state
						///////////////////////////////////////////////////////////////////////////////

						if (lowerlimb_sys_info.exercise_state == SLOWING) {
							// RESTORE
						}
					} // end if (lowerlimb_sys_info.exercise_state == RUNNING || lowerlimb_sys_info.exercise_state == SLOWING)

					///////////////////////////////////////////////////////////////////////////////
					// Invalid exercise mode:
					///////////////////////////////////////////////////////////////////////////////

					else {
						#if USE_ITM_OUT_RT_CHECK_CTRL
							printf("   <test_real_time_onepass_software()> Invalid exercise mode [%s] for activity_state == EXERCISE] \n\n", PACE_EXERC_STR[lowerlimb_sys_info.exercise_state]);
						#endif
					}
				} // end if (lowerlimb_sys_info.activity_state == EXERCISE)

				///////////////////////////////////////////////////////////////////////////////
				// HOMING activity state:
				///////////////////////////////////////////////////////////////////////////////

				else if (lowerlimb_sys_info.activity_state == HOMING) {

					// RESTORE
					}
				} // end if (lowerlimb_sys_info.activity_state == EXERCISE)

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// Set reference kinematics struct:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				// End-effector force commands:
				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				// Gravity compensation force command (end-effector coordinates):
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// FB force command:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// FF force command:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Integral position error force command:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Total force command:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Update motors' settings:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Send motor commands:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// Send feedback data to TCP/IP app:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

				///////////////////////////////////////////////////////////////////////////////
				// ITM console output:
				///////////////////////////////////////////////////////////////////////////////

				// RESTORE

			///////////////////////////////////////////////////////////////////////////////
			// Track changes of state:
			///////////////////////////////////////////////////////////////////////////////

			exercise_state_prev = lowerlimb_sys_info.exercise_state;

			cmd_code_prev = cmd_code;

			///////////////////////////////////////////////////////////////////////////////
			// ITM console output:
			///////////////////////////////////////////////////////////////////////////////

			// RESTORE

			///////////////////////////////////////////////////////////////////////////////
			// Increase real-time step counter:
			///////////////////////////////////////////////////////////////////////////////

			rt_step_i++;

		} // end if (up_time >= algo_nextTime) // RT time step
	} // end while (t_ref <= T_RUN_MAX)
}
