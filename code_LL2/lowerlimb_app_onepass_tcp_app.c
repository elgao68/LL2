/////////////////////////////////////////////////////////////////////////////
//
//  lowerlimb_app_onepass_tcp_app_ref.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "lowerlimb_app.h"

lowerlimb_sys_info_t lowerlimb_sys_info;

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// TCP/IP APP STATE:
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void
lowerlimb_app_onepass_tcp_app_ref(lowerlimb_sys_info_t* lowerlimb_sys, uint8_t ui8EBtnState, uint8_t ui8Alert, traj_ctrl_params_t* traj_ctrl_params,
		admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings, uint16_t* cmd_code_ref,
		uint8_t* calib_enc_on) {

	static uint16_t cmd_code = 0; // CRITICAL to make it static

	uint8_t tmp_index  = 0;
	uint8_t tmp_chk    = 0;

	uint64_t ui64_tmp  = 0;
	uint8_t rxPayload  = 0;
	uint8_t tmp_resp_msg[465];

	// Constants & Variables for parsing index:
	const uint8_t cmdCode_index = 3;
	const uint8_t payloadLen_index = 5;
	const uint8_t payloadStart_index = 7;

	uint8_t rx_payload_index = payloadStart_index;

	//Force sensors - TODO: remove at a later date
	#if TEST_FUNC_CALIB_FORCE_SENSORS

	#else
		uint32_t force_end_in_x_sensor = 0;
		uint32_t force_end_in_y_sensor = 0;
		uint32_t dum_force_end_in_x = 0;
		uint32_t dum_force_end_in_y = 0;
		float force_end_in_x_sensor_f = 0;
		float force_end_in_y_sensor_f = 0;
	#endif

	// Counters:
	static int step_i = 0;

	// Auxiliary variables:
	uint8_t is_valid_msg = 0;
	static uint8_t stop_exe_cmd_count = 0; // stop command counter - this is used to accommodate the SLOWING case

	////////////////////////////////////
	// Reset lower-limb system info:
	////////////////////////////////////

	lowerlimb_sys->app_status    = 0;

	////////////////////////////////////
	// Update system operation status:
	////////////////////////////////////

	update_operation_state(&lowerlimb_sys->operation_state, ui8EBtnState, ui8Alert);

	////////////////////////////////////
	// Update local Unix time:
	////////////////////////////////////

	update_unix_time();

	////////////////////////////////////
	// Check if TCP connected:
	////////////////////////////////////

	if (isTCPConnected() == 0) {
		// set activity to IDLE
		set_activity_idle();

		// set exercise_state to STOPPED
		stop_exercise(LL_motors_settings);

		// set system state to OFF
		set_system_off();

		return;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Validate self-stopping activity states (generates "virtual" cmd_code):
	////////////////////////////////////////////////////////////////////////////////

	if (lowerlimb_sys->activity_state == CALIB) {
		cmd_code = AUTO_CALIB_MODE_CMD;
		*cmd_code_ref = cmd_code;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Validate TCP messages:
	////////////////////////////////////////////////////////////////////////////////

	else {

		////////////////////////////////////////////////////////////////////////////////
		// Check if there is a valid message:
		////////////////////////////////////////////////////////////////////////////////

		if (ethernet_w5500_new_rcv_data()) {
			memset(tcpRxData, 0, sizeof(tcpRxData));
			is_valid_msg = is_valid_w5500_msg(tcpRxData);
		}
		else
			return;

		////////////////////////////////////////////////////////////////////////////////
		// Parse command code & payload:
		////////////////////////////////////////////////////////////////////////////////

		if (is_valid_msg) {
			cmd_code  = ((uint16_t) tcpRxData[cmdCode_index] << 8) +
						 (uint16_t) tcpRxData[cmdCode_index + 1];

			rxPayload = ((uint16_t) tcpRxData[payloadLen_index] << 8) +
						 (uint16_t) tcpRxData[payloadLen_index + 1];
		}
		else {
			#if USE_ITM_CMD_CHECK
				printf("   lowerlimb_app_onepass_tcp_app_ref(): is_valid_msg = [0] \n\n");
			#endif

			return;
		}

		////////////////////////////////////////////////////////////////////////////////
		// Validate command code:
		////////////////////////////////////////////////////////////////////////////////

		if (cmd_code == NO_CMD)
			return;
		else if (!is_valid_cmd_code(cmd_code, rxPayload,
				lowerlimb_sys->system_state, lowerlimb_sys->activity_state, &lowerlimb_sys->app_status)) {

			#if USE_ITM_CMD_CHECK
				printf("   lowerlimb_app_onepass_tcp_app_ref(): invalid cmd_code [%d] \n\n", cmd_code);
			#endif

			return;
		}
		else
			*cmd_code_ref = cmd_code;
	}

	// Console output:
	#if USE_ITM_CMD_CHECK
		uint8_t idx_exerc_mode  = lowerlimb_sys->exercise_mode;
		uint8_t idx_sys_state   = lowerlimb_sys->system_state;
		uint8_t idx_activ_state = lowerlimb_sys->activity_state;
		uint8_t idx_exerc_state = lowerlimb_sys->exercise_state;
	#endif

	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	// EXECUTE COMMAND CODE:
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////

	// Command codes in use:
	/*
	CMD 01: START_SYS_CMD
	CMD 02: BRAKES_CMD
	CMD 03: AUTO_CALIB_MODE_CMD
	CMD 04: START_RESUME_EXE_CMD
	CMD 05: STOP_EXE_CMD
	CMD 06: STOP_SYS_CMD
	*/

	#if USE_ITM_CMD_CHECK
		if (cmd_code == START_SYS_CMD 			||
			cmd_code == BRAKES_CMD 				||
			// cmd_code == AUTO_CALIB_MODE_CMD 	||
			cmd_code == START_RESUME_EXE_CMD 	||
			cmd_code == STOP_EXE_CMD 			||
			cmd_code == STOP_SYS_CMD) {
				printf("   ____________________________\n");
				printf("   cmd_code(%d) [%s] \n", cmd_code, CMD_STR[cmd_code]);
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf(" \n");
		}
	#endif

	///////////////////////////////////////////////////////////////////////////
	// CMD 01: START_SYS_CMD
	///////////////////////////////////////////////////////////////////////////

	if (cmd_code == START_SYS_CMD) { //start system

		reset_lowerlimb_sys_info();

		lowerlimb_sys->system_state   = SYS_ON;
		lowerlimb_sys->activity_state = IDLE;
		lowerlimb_sys->exercise_state = STOPPED;

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD 02: BRAKES_CMD
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == BRAKES_CMD) {

		if (tcpRxData[rx_payload_index] == 0x01) {
			lowerlimb_brakes_command.l_brake_disengage = true;
			lowerlimb_brakes_command.r_brake_disengage = true;

			l_brakes(DISENGAGE_BRAKES);
			r_brakes(DISENGAGE_BRAKES);
		}
		else {
			lowerlimb_brakes_command.l_brake_disengage = false;
			lowerlimb_brakes_command.r_brake_disengage = false;

			l_brakes(ENGAGE_BRAKES);
			r_brakes(ENGAGE_BRAKES);
		}

		send_OK_resp(cmd_code);

		#if USE_ITM_CMD_CHECK
			printf("   BRAKES_CMD data: [%d]\n", tcpRxData[rx_payload_index]);
			printf("   brakes disengaged: [");
			if (lowerlimb_brakes_command.l_brake_disengage)
				printf("TRUE] \n\n");
			else
				printf("FALSE] \n\n");
		#endif
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD 03: AUTO_CALIB_MODE_CMD
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == AUTO_CALIB_MODE_CMD) { //enter calibration

		///////////////////////////////////////////////////////////////////////////
		// Update activity state to CALIB, otherwise check for errors:
		///////////////////////////////////////////////////////////////////////////

		if (lowerlimb_sys->activity_state == IDLE) {

			// Update activity state:
			lowerlimb_sys->activity_state = CALIB;

			#if USE_ITM_CMD_CHECK
				idx_sys_state   = lowerlimb_sys->system_state;
				idx_activ_state = lowerlimb_sys->activity_state;
				idx_exerc_state = lowerlimb_sys->exercise_state;

				printf("   INTERMEDIATE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

			// Zero-calibrate force sensors:
			#if TEST_FUNC_CALIB_FORCE_SENSORS
				force_sensors_zero_calibrate(&hadc3);
			#else
				// TODO: remove at a later date:
				for (int i = 1; i <= 50; i++) {
					force_sensors_read(&hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor,
							&dum_force_end_in_x, &dum_force_end_in_y);

					force_end_in_x_sensor_f += (float) force_end_in_x_sensor * 3.3f / 4095.0f;
					force_end_in_y_sensor_f += (float) force_end_in_y_sensor * 3.3f / 4095.0f;
				}

				force_end_in_x_sensor_f = force_end_in_x_sensor_f / 50.0f;
				force_end_in_y_sensor_f = force_end_in_y_sensor_f / 50.0f;

				set_force_sensor_zero_offset(force_end_in_x_sensor_f, force_end_in_y_sensor_f);
			#endif

			#if USE_ITM_CMD_CHECK
				printf("   <<lowerlimb_app_onepass_tcp_app_ref()>> [Force sensors calibrated] \n\n");
			#endif

			// Reset encoders - CRITICAL:
			qei_count_L_reset();
			qei_count_R_reset();

			// Activate encoder calibration:
			*calib_enc_on = 1;
		}

		// TODO: remove at a later date - Check for errors:
		else {
			// _VALIDATE_CMD_CALIB
		}

		///////////////////////////////////////////////////////////////////////////
		// Perform calibration per requisites in lowerlimb_sys->:
		///////////////////////////////////////////////////////////////////////////

		if (lowerlimb_sys->activity_state == CALIB && *calib_enc_on == 0) { // NOTE: *calib_enc_on is only zero'd by test_real_time_control()

			#if USE_ITM_CMD_CHECK
				printf("   <<lowerlimb_app_onepass_tcp_app_ref()>> [Encoders calibrated] \n\n");
			#endif

			// Reset activity to IDLE:
			lowerlimb_sys->activity_state = IDLE;

			send_OK_resp(cmd_code);
		}
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD 04: START_RESUME_EXE_CMD
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == START_RESUME_EXE_CMD) {

		// Reset emergency alerts if any:
		reset_emergency_alerts();

		// If activity is in IDLE state, start exercise to SETUP state:
		if (lowerlimb_sys->activity_state == IDLE) {
			// Validate command code:
			_VALIDATE_IDLE_START_EXE

			// Set activity to exercise:
			lowerlimb_sys->activity_state = EXERCISE;

			// Exercise state goes to SETUP:
			lowerlimb_sys->exercise_state = SETUP; // is there a way we can switch to RUNNING directly?
		}
		else {
			// _VALIDATE_CMD_START_EXE
		}

		send_OK_resp(cmd_code);

		#if USE_ITM_CMD_CHECK
			idx_exerc_mode  = lowerlimb_sys->exercise_mode;
			printf("   exercise mode = [%s] \n\n", EXERC_MODE_STR[idx_exerc_mode]);
		#endif
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD 05: STOP_EXE_CMD
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == STOP_EXE_CMD) {

		lowerlimb_sys->exercise_state = SLOWING;

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD 06: STOP_SYS_CMD
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == STOP_SYS_CMD) { // stop system
		lowerlimb_sys->system_state   = SYS_OFF;
		lowerlimb_sys->activity_state = IDLE;
		lowerlimb_sys->exercise_state = STOPPED;

		//Reset brake command
		lowerlimb_brakes_command.l_brake_disengage = false;
		lowerlimb_brakes_command.r_brake_disengage = false;

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Unknown command:
	///////////////////////////////////////////////////////////////////////////

	else {
		// tx unknown command error:
		send_error_msg(cmd_code, ERR_UNKNOWN);
		lowerlimb_sys->app_status = ERR_UNKNOWN + 3;

		#if USE_ITM_CMD_CHECK
			printf("   <<lowerlimb_app_onepass_tcp_app_ref()>> cmd_code(%d) UNKNOWN \n", cmd_code);
			printf("\n");
		#endif
		return;
	}

	#if USE_ITM_CMD_CHECK
		if (cmd_code == START_SYS_CMD 			||
			cmd_code == BRAKES_CMD 				||
			// cmd_code == AUTO_CALIB_MODE_CMD 	||
			cmd_code == START_RESUME_EXE_CMD 	||
			cmd_code == STOP_EXE_CMD 			||
			cmd_code == STOP_SYS_CMD) {

				idx_sys_state   = lowerlimb_sys->system_state;
				idx_activ_state = lowerlimb_sys->activity_state;
				idx_exerc_state = lowerlimb_sys->exercise_state;

				printf("   AFTER: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf(" \n");
		}
	#endif
}
