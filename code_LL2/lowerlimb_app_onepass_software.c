/////////////////////////////////////////////////////////////////////////////
//
// lowerlimb_app_onepass_software.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "lowerlimb_app.h"

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// TCP/IP APP STATE:
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

lowerlimb_sys_info_t
lowerlimb_app_onepass_software(uint8_t ui8EBtnState, uint8_t ui8Alert, traj_ctrl_params_t* traj_ctrl_params,
		admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings, uint16_t* cmd_code_last,
		uint8_t* calib_enc_on, uint8_t* homing_on) {

	static uint16_t cmd_code = 0; // CRITICAL to make it static

	uint8_t tmp_index  = 0;
	uint8_t tmp_chk    = 0;

	uint64_t ui64_tmp  = 0;
	uint8_t rxPayload  = 0;
	uint8_t tmp_resp_msg[465];

	////////////////////////////////////
	// Constants & variables for parsing index:
	////////////////////////////////////

	const uint8_t cmdCode_index      = 3;
	const uint8_t payloadLen_index   = 5;
	const uint8_t payloadStart_index = 7;

	uint8_t rx_payload_index = payloadStart_index;

	tcp_exercise_targ_params_t tmp_parsed_targ_params;

	////////////////////////////////////
	// Force Sensor:
	////////////////////////////////////

	uint32_t force_end_in_x_sensor = 0;
	uint32_t force_end_in_y_sensor = 0;
	uint32_t dum_force_end_in_x    = 0;
	uint32_t dum_force_end_in_y    = 0;
	float force_end_in_x_sensor_f  = 0;
	float force_end_in_y_sensor_f  = 0;

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

	lowerlimb_sys_info.app_status    = 0;

	////////////////////////////////////
	// Update system operation status:
	////////////////////////////////////

	update_operation_state(&lowerlimb_sys_info.operation_state, ui8EBtnState, ui8Alert);

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

		return lowerlimb_sys_info;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Validate TCP messages:
	////////////////////////////////////////////////////////////////////////////////

	// if (lowerlimb_sys_info.activity_state != CALIB) { // see persistent_activ_state

		////////////////////////////////////////////////////////////////////////////////
		// Check if there is a valid message:
		////////////////////////////////////////////////////////////////////////////////

		if (ethernet_w5500_new_rcv_data()) {
			memset(tcpRxData, 0, sizeof(tcpRxData));
			is_valid_msg = is_valid_w5500_msg(tcpRxData);
		}
		else
			return lowerlimb_sys_info;

		////////////////////////////////////////////////////////////////////////////////
		// Parse command code & payload:
		////////////////////////////////////////////////////////////////////////////////

		if (is_valid_msg) {
			cmd_code =  ((uint16_t) tcpRxData[cmdCode_index] << 8) +
						 (uint16_t) tcpRxData[cmdCode_index + 1];

			rxPayload = ((uint16_t) tcpRxData[payloadLen_index] << 8) +
						 (uint16_t) tcpRxData[payloadLen_index + 1];
		}
		else {
			#if USE_ITM_CMD_CHECK
				printf("   <<lowerlimb_app_onepass_software()>>: is_valid_msg = [0] \n\n");
			#endif

			return lowerlimb_sys_info;
		}

		////////////////////////////////////////////////////////////////////////////////
		// Validate command code:
		////////////////////////////////////////////////////////////////////////////////

		if (is_valid_msg_tcp_payload_size(cmd_code, rxPayload, &lowerlimb_sys_info.app_status)) {
			*cmd_code_last = cmd_code;
		}
		else {
			#if USE_ITM_CMD_CHECK
				printf("   <<lowerlimb_app_onepass_software()>>: invalid cmd_code [%d] payload \n\n", cmd_code);
			#endif

			return lowerlimb_sys_info;
		}

	// }

	// Console output:
	#if USE_ITM_CMD_CHECK
		uint8_t idx_exerc_mode  = lowerlimb_sys_info.exercise_mode;
		uint8_t idx_sys_state   = lowerlimb_sys_info.system_state;
		uint8_t idx_activ_state = lowerlimb_sys_info.activity_state;
		uint8_t idx_exerc_state = lowerlimb_sys_info.exercise_state;
	#endif

	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	// EXECUTE COMMAND CODE (STATE MACHINE codes):
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////

	// State-machine command codes in use:
	/*
	_0_MSG_TCP 					= 0,
	_1_MSG_TCP 					= 1,
	_2_MSG_TCP					= 2,
	Connect_To_Robot_MSG_TCP	= 3,
	Calibrate_Robot_MSG_TCP		= 4,
	Move_To_Start_MSG_TCP		= 5,
	_6_MSG_TCP					= 6,
	Go_To_Exercise_MSG_TCP		= 7,
	Pedal_Travel_MSG_TCP		= 8,
	Robot_Shutdown_MSG_TCP		= 9,
	F_Therapy_Change_MSG_TCP	= 10,
	Start_Exercise_MSG_TCP		= 11,
	Stop_Exercise_MSG_TCP		= 12,
	Stdby_Start_Point_MSG_TCP	= 13
	*/

	#if USE_ITM_CMD_CHECK
		printf("____________________________________\n");
		if (cmd_code >= _0_MSG_TCP && cmd_code <= Stdby_Start_Point_MSG_TCP)
			printf("MSG_TCP (%d) [%s] \n", cmd_code, MSG_TCP_STR[cmd_code]);
		else
			printf("MSG_TCP (%d): INVALID cmd_code \n", cmd_code);
	#endif

	///////////////////////////////////////////////////////////////////////////
	// Connect_To_Robot	 = 3 (CMD 01, CMD 02)
	///////////////////////////////////////////////////////////////////////////

	if (cmd_code == Connect_To_Robot_MSG_TCP) { //connect robot

		reset_lowerlimb_sys_info();

		lowerlimb_sys_info.system_state   = SYS_ON;
		lowerlimb_sys_info.activity_state = IDLE;
		lowerlimb_sys_info.exercise_state = STOPPED;

		// HARDWARE: disengage brakes:
		/*
		lowerlimb_brakes_command.l_brake_disengage = true;
		lowerlimb_brakes_command.r_brake_disengage = true;

		l_brakes(DISENGAGE_BRAKES);
		r_brakes(DISENGAGE_BRAKES);
		*/

		send_OK_resp(cmd_code);

		#if USE_ITM_CMD_CHECK
			printf("   system_state = [%s]\n\n", SYS_STATE_STR[lowerlimb_sys_info.system_state]);
			printf("   brakes disengaged: [");
			if (lowerlimb_brakes_command.l_brake_disengage)
				printf("TRUE] \n\n");
			else
				printf("FALSE] \n\n");
		#endif
	}

	///////////////////////////////////////////////////////////////////////////
	// Calibrate_Robot	 = 4 (CMD 03)
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Calibrate_Robot_MSG_TCP) { //calibrate robot

		#if USE_ITM_CMD_CHECK
			static uint8_t init_calib_itm = 1;

			if (init_calib_itm) {
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");

				init_calib_itm = 0;
			}
		#endif

		///////////////////////////////////////////////////////////////////////////
		// Update activity state to CALIB & calibrate force sensors, otherwise check for errors:
		///////////////////////////////////////////////////////////////////////////

		// if (lowerlimb_sys_info.activity_state == IDLE) {

			// Update activity state:
			lowerlimb_sys_info.activity_state = CALIB;

			#if USE_ITM_CMD_CHECK
				idx_sys_state   = lowerlimb_sys_info.system_state;
				idx_activ_state = lowerlimb_sys_info.activity_state;
				idx_exerc_state = lowerlimb_sys_info.exercise_state;

				printf("   INTERMEDIATE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

			// HARDWARE: zero-calibrate force sensor:
			/*
			for (int i = 1; i <= 50; i++) {
				force_sensors_read(&hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor,
						&dum_force_end_in_x, &dum_force_end_in_y);

				force_end_in_x_sensor_f += (float) force_end_in_x_sensor * 3.3f / 4095.0f;
				force_end_in_y_sensor_f += (float) force_end_in_y_sensor * 3.3f / 4095.0f;
			}

			force_end_in_x_sensor_f = force_end_in_x_sensor_f / 50.0f;
			force_end_in_y_sensor_f = force_end_in_y_sensor_f / 50.0f;

			set_force_sensor_zero_offset(force_end_in_x_sensor_f, force_end_in_y_sensor_f);
			*/

			#if USE_ITM_CMD_CHECK
				printf("   <<lowerlimb_app_onepass_software()>>: [Force sensors calibrated] \n\n");
			#endif

			// HARDWARE: reset encoders - CRITICAL:
			/*
			qei_count_L_reset();
			qei_count_R_reset();
			*/

			// Activate encoder calibration:
			*calib_enc_on = 1;
		/*
		}

		// Check for errors:
		else {
			// _VALIDATE_CMD_CALIB
		}
		*/

		///////////////////////////////////////////////////////////////////////////
		// Check for encoders calibration in real-time process:
		///////////////////////////////////////////////////////////////////////////

		*calib_enc_on = 0; // HACK: this flag should be updated by the real-time control process

		if (lowerlimb_sys_info.activity_state == CALIB && *calib_enc_on == 0) { // NOTE: *calib_enc_on is only zero'd by test_real_time_control()

			// Reset activity to IDLE:
			lowerlimb_sys_info.activity_state = IDLE;

			send_OK_resp(cmd_code); // CRITICAL for software app to continue

			#if USE_ITM_CMD_CHECK
				printf("   <<lowerlimb_app_onepass_software()>>: [Encoders calibrated] \n\n");

				idx_sys_state   = lowerlimb_sys_info.system_state;
				idx_activ_state = lowerlimb_sys_info.activity_state;
				idx_exerc_state = lowerlimb_sys_info.exercise_state;

				printf("   AFTER: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif
		}
	}

	///////////////////////////////////////////////////////////////////////////
	// Move_To_Start	 = 5
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Move_To_Start_MSG_TCP) { //move to start point

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Go_To_Exercise	 = 7
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Go_To_Exercise_MSG_TCP) { //set Move to start

		// PARSE TARGET PARAMETERS:
		// tmp_parsed_targ_params.index = tcpRxData[rx_payload_index];
		// rx_payload_index += sizeof(tmp_parsed_targ_params.index);

		memset(&tmp_parsed_targ_params, 0, sizeof(tmp_parsed_targ_params));

		memcpy_msb(&tmp_parsed_targ_params.EX_MODE, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.EX_MODE));
		rx_payload_index += sizeof(tmp_parsed_targ_params.EX_MODE);

		memcpy_msb(&tmp_parsed_targ_params.EX_TYPE, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.EX_TYPE));
		rx_payload_index += sizeof(tmp_parsed_targ_params.EX_TYPE);

		memcpy_msb(&tmp_parsed_targ_params.speed_ex, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.speed_ex));
		rx_payload_index += sizeof(tmp_parsed_targ_params.speed_ex);

		memcpy_msb(&tmp_parsed_targ_params.point_ext_start, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.point_ext_start));
		rx_payload_index += sizeof(tmp_parsed_targ_params.point_ext_start);

		memcpy_msb(&tmp_parsed_targ_params.point_ext_end, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.point_ext_end));
		rx_payload_index += sizeof(tmp_parsed_targ_params.point_ext_end);

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Pedal_Travel			= 8
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Pedal_Travel_MSG_TCP) { //Move Pedal

		// PARSE TARGET PARAMETERS:
		memset(&tmp_parsed_targ_params, 0, sizeof(tmp_parsed_targ_params));

		memcpy_msb(&tmp_parsed_targ_params.dist_x, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.dist_x));
		rx_payload_index += sizeof(tmp_parsed_targ_params.dist_x);

		memcpy_msb(&tmp_parsed_targ_params.dist_y, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.dist_y));
		rx_payload_index += sizeof(tmp_parsed_targ_params.dist_y);

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Robot_Shutdown	 = 9 (CMD 06)
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Robot_Shutdown_MSG_TCP) { //shutdown robot

		/*
		if (lowerlimb_sys_info.system_state != SYS_ON){
			send_error_msg(cmd_code, ERR_GENERAL_NOK);
			lowerlimb_sys_info.app_status = ERR_GENERAL_NOK + 3;
			return lowerlimb_sys_info;
		}
		*/

		lowerlimb_sys_info.system_state   = SYS_OFF;
		lowerlimb_sys_info.activity_state = IDLE;
		lowerlimb_sys_info.exercise_state = STOPPED;

		// HARDWARE: engage brakes
		/*
		lowerlimb_brakes_command.l_brake_disengage = false;
		lowerlimb_brakes_command.r_brake_disengage = false;
		*/

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// F_Therapy_Change	= 10
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == F_Therapy_Change_MSG_TCP) { //Change F_therapy

		// PARSE TARGET PARAMETERS:
		memset(&tmp_parsed_targ_params, 0, sizeof(tmp_parsed_targ_params));

		memcpy_msb(&tmp_parsed_targ_params.F_Therapy_change, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_targ_params.F_Therapy_change));
		rx_payload_index += sizeof(tmp_parsed_targ_params.F_Therapy_change);

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Start_Exercise	 = 11 (CMD 04)
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Start_Exercise_MSG_TCP) { //start exercise

		#if USE_ITM_CMD_CHECK
			printf("   BEFORE: \n");
			printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
			printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
			printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
			printf("\n");
		#endif

		// Reset emergency alerts if any:
		reset_emergency_alerts();

		// If activity is in IDLE state, start exercise to SETUP state:
		if (lowerlimb_sys_info.activity_state == IDLE) {

			// Validate command code:
			_VALIDATE_IDLE_START_EXE

			// Set activity to exercise:
			lowerlimb_sys_info.activity_state = EXERCISE;

			// Exercise state goes to SETUP:
			lowerlimb_sys_info.exercise_state = SETUP; // can we switch to RUNNING directly?
		}
		else {
			// _VALIDATE_CMD_START_EXE
		}

		send_OK_resp(cmd_code);

		#if USE_ITM_CMD_CHECK
			idx_exerc_mode  = lowerlimb_sys_info.exercise_mode;
			idx_sys_state   = lowerlimb_sys_info.system_state;
			idx_activ_state = lowerlimb_sys_info.activity_state;
			idx_exerc_state = lowerlimb_sys_info.exercise_state;

			printf("   exercise mode = [%s] \n\n", EXERC_MODE_STR[idx_exerc_mode]);

			printf("   AFTER: \n");
			printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
			printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
			printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
			printf("\n");
		#endif
	}
	///////////////////////////////////////////////////////////////////////////
	// Stop_Exercise	 = 12 (CMD 05)
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Stop_Exercise_MSG_TCP) { //stop exercise

		lowerlimb_sys_info.exercise_state = SLOWING;

		send_OK_resp(cmd_code);

		#if USE_ITM_CMD_CHECK
			idx_sys_state   = lowerlimb_sys_info.system_state;
			idx_activ_state = lowerlimb_sys_info.activity_state;
			idx_exerc_state = lowerlimb_sys_info.exercise_state;

			printf("   AFTER: \n");
			printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
			printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
			printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
			printf("\n");
		#endif
	}

	///////////////////////////////////////////////////////////////////////////
	// Stdby_Start_Point	= 13
	///////////////////////////////////////////////////////////////////////////

	else if (cmd_code == Stdby_Start_Point_MSG_TCP) { //standby start point

		send_OK_resp(cmd_code);
	}

	///////////////////////////////////////////////////////////////////////////
	// Unknown command:
	///////////////////////////////////////////////////////////////////////////

	else {
		// TODO: reinstate this response if necessary for the software app:
		/*
		send_error_msg(cmd_code, ERR_UNKNOWN);
		lowerlimb_sys_info.app_status = ERR_UNKNOWN + 3;
		*/
	}

	return lowerlimb_sys_info;
}
