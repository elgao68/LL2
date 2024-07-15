/////////////////////////////////////////////////////////////////////////////
//
//  lowerlimb_app_onepass_ref.c
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
lowerlimb_app_onepass_ref(lowerlimb_sys_info_t* lowerlimb_sys, uint8_t ui8EBtnState, uint8_t ui8Alert, traj_ctrl_params_t* traj_ctrl_params,
		admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings, uint16_t* cmd_code_ref,
		uint8_t* calib_enc_on, uint8_t use_software_msg_list) {

	////////////////////////////////////
	// Messaging variables:
	////////////////////////////////////

	// Select TCP messages list to use (CRITICAL):
	__SELECT_CMD_CODE_LIST(use_software_msg_list)

	static uint16_t cmd_code = 0; // CRITICAL to make it static

	uint8_t tmp_index  = 0;
	uint8_t tmp_chk    = 0;

	uint64_t ui64_tmp  = 0;
	uint8_t rxPayload  = 0;
	uint8_t tmp_resp_msg[465];

	// Constants & variables for parsing index:
	const uint8_t cmdCode_index = 3;
	const uint8_t payloadLen_index = 5;
	const uint8_t payloadStart_index = 7;

	uint8_t rx_payload_index = payloadStart_index;

	////////////////////////////////////
	// Counters:
	////////////////////////////////////

	static int step_i = 0;

	////////////////////////////////////
	// Auxiliary variables:
	////////////////////////////////////

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
		cmd_code = _CALIBRATE;
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
			#if USE_ITM_CMD_DISPLAY
				printf("   <<lowerlimb_app_onepass_ref()>> is_valid_msg = [0] \n\n");
			#endif

			return;
		}

		////////////////////////////////////////////////////////////////////////////////
		// Validate command code (NOTE: 'is_valid' functions reject a zero cmd_code (NO_CMD / NO_MSG_TCP) as invalid):
		////////////////////////////////////////////////////////////////////////////////

		if (!use_software_msg_list &&
				is_valid_cmd_code_tcp_app(cmd_code, rxPayload,
						lowerlimb_sys->system_state, lowerlimb_sys->activity_state, &lowerlimb_sys->app_status))
							*cmd_code_ref = cmd_code;

		else if (use_software_msg_list &&
				is_valid_payload_size_software(cmd_code, rxPayload, &lowerlimb_sys_info.app_status)) // Software-generated messages: check only payload, not states
							*cmd_code_ref = cmd_code;

		else {
			#if USE_ITM_CMD_DISPLAY
				printf("   <<lowerlimb_app_onepass_ref()>> use_software_msg_list = [%d], invalid cmd_code [%d] \n\n", use_software_msg_list, cmd_code);
			#endif

			return;
		}
	}

	// Console output:
	#if USE_ITM_CMD_DISPLAY
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

	// Generic command codes:
	/*
	_START_SYSTEM
	_BRAKES_ON_OFF
	_CALIBRATE
	_START_EXERCISE
	_STOP_EXERCISE
	_STOP_SYSTEM
	_MOVE_TO_START
	_GO_TO_EXERCISE
	_PEDAL_TRAVEL
	_FORCE_THERAPY_CHANGE
	_STDBY_START_POINT
	*/

	#if USE_ITM_CMD_DISPLAY
		// NOTE: _CALIBRATE behaves differently
		if (cmd_code == _START_SYSTEM 		||
			cmd_code == _BRAKES_ON_OFF 		||
			cmd_code == _START_EXERCISE 	||
			cmd_code == _STOP_EXERCISE 		||
			cmd_code == _STOP_SYSTEM) {
				printf("   ____________________________\n");
				if (use_software_msg_list)
					printf("   cmd_code(%d) [%s] \n", cmd_code, MSG_TCP_STR[cmd_code]);
				else
					printf("   cmd_code(%d) [%s] \n", cmd_code, CMD_STR[cmd_code]);
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", MODE_EXERC_STR[idx_exerc_state]);
				printf(" \n");
		}
	#endif

	///////////////////////////////////////////////////////////////////////////
	// CMD A: _START_SYSTEM
	///////////////////////////////////////////////////////////////////////////

	if (cmd_code == _START_SYSTEM) { //start system

		reset_lowerlimb_sys_info();

		lowerlimb_sys->system_state   = SYS_ON;
		lowerlimb_sys->activity_state = IDLE;
		lowerlimb_sys->exercise_state = STOPPED;

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD B: _BRAKES_ON_OFF
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == _BRAKES_ON_OFF) {

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

		#if USE_ITM_CMD_DISPLAY
			printf("   _BRAKES_ON_OFF data: [%d]\n", tcpRxData[rx_payload_index]);
			printf("   brakes disengaged: [");
			if (lowerlimb_brakes_command.l_brake_disengage)
				printf("TRUE] \n\n");
			else
				printf("FALSE] \n\n");
		#endif
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD C: _CALIBRATE
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == _CALIBRATE) { //enter calibration

		///////////////////////////////////////////////////////////////////////////
		// Update activity state to CALIB, otherwise check for errors:
		///////////////////////////////////////////////////////////////////////////

		if (lowerlimb_sys->activity_state == IDLE) {

			// Update activity state:
			lowerlimb_sys->activity_state = CALIB;

			#if USE_ITM_CMD_DISPLAY
				idx_sys_state   = lowerlimb_sys->system_state;
				idx_activ_state = lowerlimb_sys->activity_state;
				idx_exerc_state = lowerlimb_sys->exercise_state;

				printf("   INTERMEDIATE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", MODE_EXERC_STR[idx_exerc_state]);
				printf("\n");
			#endif

			// Zero-calibrate force sensors:
			force_sensors_zero_calibrate(&hadc3);

			#if USE_ITM_CMD_DISPLAY
				printf("   <<lowerlimb_app_onepass_ref()>> [Force sensors calibrated] \n\n");
			#endif

			// Reset encoders - CRITICAL:
			qei_count_L_reset();
			qei_count_R_reset();

			// Activate encoder calibration:
			*calib_enc_on = 1;
		}

		// TODO: remove at a later date
		/*
		else {
			__VALIDATE_CMD_CALIB
		}
		*/

		///////////////////////////////////////////////////////////////////////////
		// Perform calibration per requisites in lowerlimb_sys->:
		///////////////////////////////////////////////////////////////////////////

		if (lowerlimb_sys->activity_state == CALIB && *calib_enc_on == 0) { // NOTE: *calib_enc_on is only zero'd by test_real_time_onepass_control()

			#if USE_ITM_CMD_DISPLAY
				printf("   <<lowerlimb_app_onepass_ref()>> [Encoders calibrated] \n\n");
			#endif

			// Reset activity to IDLE:
			lowerlimb_sys->activity_state = IDLE;

			send_OK_resp(cmd_code);
		}
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD D: _START_EXERCISE
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == _START_EXERCISE) {

		// Reset emergency alerts if any:
		reset_emergency_alerts();

		// If activity is in IDLE state, start exercise to SETUP state:
		if (lowerlimb_sys->activity_state == IDLE) {
			// Validate command code:
			__VALIDATE_IDLE_START_EXE

			// Set activity to exercise:
			lowerlimb_sys->activity_state = EXERCISE;

			// Exercise state goes to SETUP:
			lowerlimb_sys->exercise_state = SETUP; // is there a way we can switch to RUNNING directly?
		}
		// TODO: remove at a later date
		/*
		else {
			__VALIDATE_CMD_START_EXE
		}
		*/

		send_OK_resp(cmd_code);

		#if USE_ITM_CMD_DISPLAY
			idx_exerc_mode  = lowerlimb_sys->exercise_mode;
			printf("   exercise mode = [%s] \n\n", EXERC_MODE_STR[idx_exerc_mode]);
		#endif
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD E: _STOP_EXERCISE
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == _STOP_EXERCISE) {

		lowerlimb_sys->exercise_state = SLOWING;

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// CMD F: _STOP_SYSTEM
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == _STOP_SYSTEM) { // stop system
		lowerlimb_sys->system_state   = SYS_OFF;
		lowerlimb_sys->activity_state = IDLE;
		lowerlimb_sys->exercise_state = STOPPED;

		//Reset brake command
		lowerlimb_brakes_command.l_brake_disengage = false;
		lowerlimb_brakes_command.r_brake_disengage = false;

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Other commands:
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == _MOVE_TO_START 		||
			 cmd_code == _GO_TO_EXERCISE 		||
			 cmd_code == _PEDAL_TRAVEL 			||
			 cmd_code == _FORCE_THERAPY_CHANGE	||
			 cmd_code == _STDBY_START_POINT) {
				#if USE_ITM_CMD_DISPLAY
					printf("   ----------------------------\n");
					if (use_software_msg_list)
						printf("   cmd_code(%d) [%s] \n", cmd_code, MSG_TCP_STR[cmd_code]);
					else
						printf("   cmd_code(%d) [%s] \n", cmd_code, CMD_STR[cmd_code]);
				#endif

				send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Unknown command:
	///////////////////////////////////////////////////////////////////////////

	else {
		// Unknown command error:
		send_error_msg(cmd_code, ERR_UNKNOWN);
		lowerlimb_sys->app_status = ERR_UNKNOWN + 3;

		#if USE_ITM_CMD_DISPLAY
			printf("   <<lowerlimb_app_onepass_ref()>> cmd_code(%d) UNKNOWN \n", cmd_code);
			printf("\n");
		#endif
		return;
	}

	#if USE_ITM_CMD_DISPLAY
		// NOTE: _CALIBRATE behaves differently (2)
		if (cmd_code == _START_SYSTEM 		||
			cmd_code == _BRAKES_ON_OFF 		||
			cmd_code == _START_EXERCISE 	||
			cmd_code == _STOP_EXERCISE 		||
			cmd_code == _STOP_SYSTEM) {

				idx_sys_state   = lowerlimb_sys->system_state;
				idx_activ_state = lowerlimb_sys->activity_state;
				idx_exerc_state = lowerlimb_sys->exercise_state;

				printf("   AFTER: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", MODE_EXERC_STR[idx_exerc_state]);
				printf(" \n");
		}
	#endif
}
