/////////////////////////////////////////////////////////////////////////////
//
//  lowerlimb_app_scripts_tcpip.c
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
lowerlimb_app_state_tcpip(uint8_t ui8EBtnState, uint8_t ui8Alert, traj_ctrl_params_t* traj_ctrl_params,
		admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings, uint16_t* cmd_code_copy,
		uint8_t* calib_fsens_on, uint8_t* calib_enc_on ) {

	// uint16_t n_cnt     = 0; // delete
	// uint8_t tmp_chksum = 0; // delete
	uint8_t tmp_index  = 0;
	uint8_t tmp_chk    = 0;

	uint16_t cmd_code  = 0;
	static uint16_t cmd_code_prev = 0;

	uint64_t ui64_tmp  = 0;
	uint8_t rxPayload  = 0;
	uint8_t tmp_resp_msg[465];
	//int16_t i6TmpQEIL, i6TmpQEIR;

	// Constants & Variables for parsing index:
	// const uint8_t startByte0_index = 0; // delete
	// const uint8_t startByte1_index = 1; // delete
	// const uint8_t msgId_index = 2; // delete
	const uint8_t cmdCode_index = 3;
	const uint8_t payloadLen_index = 5;
	const uint8_t payloadStart_index = 7;
	// uint8_t checkSum_index; // delete
	// uint8_t endByte0_index; // delete
	// uint8_t endByte1_index; // delete

	uint8_t rx_payload_index = payloadStart_index;
	// uint16_t resp_payload_index = 0;
	// const uint8_t exercise_payload_length = 89;

	//Force Sensor:
	uint32_t force_end_in_x_sensor = 0;
	uint32_t force_end_in_y_sensor = 0;
	uint32_t dum_force_end_in_x = 0;
	uint32_t dum_force_end_in_y = 0;
	float force_end_in_x_sensor_f = 0;
	float force_end_in_y_sensor_f = 0;

	// Counters:
	static int step_i = 0;

	// Auxiliary variables:
	uint8_t is_valid_msg = 0;
	uint8_t is_valid_cmd = 0;
	static uint8_t stop_exe_cmd_count = 0; // stop command counter - this is used to accommodate the SLOWING case

	// Display variables:
	#if USE_ITM_CMD_CHECK
		uint8_t idx_exerc_mode;
		uint8_t idx_sys_state;
		uint8_t idx_activ_state;
		uint8_t idx_exerc_state;
	#endif

	////////////////////////////////////
	// Reset lower-limb system info:
	////////////////////////////////////

	lowerlimb_sys_info.app_status    = 0; // delete
	// lowerlimb_sys_info.calib_prot_req = 0; // delete

	////////////////////////////////////
	// Update system operation status:
	////////////////////////////////////

	update_operation_state(&lowerlimb_sys_info.operation_state, ui8EBtnState, ui8Alert);

	/*
	if (lowerlimb_sys_info.operation_state == 0)
		lowerlimb_sys_info.operation_state |= OPS_STATUS_NORMAL_MASK;

	if (ui8Alert == 1) {
		lowerlimb_sys_info.operation_state |= OPS_STATUS_SAMPLING_ALERT_MASK;
		lowerlimb_sys_info.operation_state &= ~OPS_STATUS_NORMAL_MASK;
	} else if (ui8Alert == 2) {
		lowerlimb_sys_info.operation_state |= OPS_STATUS_RT_VEL_ALERT_MASK;
		lowerlimb_sys_info.operation_state &= ~OPS_STATUS_NORMAL_MASK;
	}

	if (ui8EBtnState == 0)
		lowerlimb_sys_info.operation_state |= OPS_STATUS_EBTN_PRESSED_MASK;
	else
		lowerlimb_sys_info.operation_state &= ~OPS_STATUS_EBTN_PRESSED_MASK;
	*/

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

	////////////////////////////////////
	// Parsing and executing TCP messages
	////////////////////////////////////

	// Check if there's new message:
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
		cmd_code = ((uint16_t) tcpRxData[cmdCode_index] << 8) +
					(uint16_t) tcpRxData[cmdCode_index + 1];

		rxPayload = ((uint16_t) tcpRxData[payloadLen_index] << 8) +
					 (uint16_t) tcpRxData[payloadLen_index + 1];
	}
	else
		return lowerlimb_sys_info;

	*cmd_code_copy = cmd_code;

	// Console output:
	/*
	#if USE_ITM_CMD_CHECK
		if (cmd_code != cmd_code_prev) {
			printf("\n");
			printf("   lowerlimb_app_state_tcpip():\n");
			printf("   ON ethernet_w5500_new_rcv_data(): step_i [%d], cmd_code_prev = [%s], cmd_code = [%s]\n\n", step_i, CMD_STR[cmd_code_prev], CMD_STR[cmd_code]);

		}
		step_i++;
	#endif
	*/

	////////////////////////////////////////////////////////////////////////////////
	// VALIDATE COMMAND CODE:
	////////////////////////////////////////////////////////////////////////////////

	is_valid_cmd = is_valid_cmd_code(cmd_code, rxPayload,
			lowerlimb_sys_info.system_state, lowerlimb_sys_info.activity_state, &lowerlimb_sys_info.app_status);

	if (!is_valid_cmd)
		return lowerlimb_sys_info;

	// Console output:
	#if USE_ITM_CMD_CHECK
		idx_exerc_mode  = lowerlimb_sys_info.exercise_mode;
		idx_sys_state   = lowerlimb_sys_info.system_state;
		idx_activ_state = lowerlimb_sys_info.activity_state;
		idx_exerc_state = lowerlimb_sys_info.exercise_state;
	#endif

	/*
	// Payload size: 1
	if (cmd_code == START_SYS_CMD		||
		cmd_code == BRAKES_CMD			||
		cmd_code == AUTO_CALIB_MODE_CMD ||
		cmd_code == STOP_SYS_CMD		||
		cmd_code == STOP_EXE_CMD		||
		cmd_code == RESET_SYS_CMD		||
		cmd_code == READ_DEV_ID_CMD		||
		cmd_code == READ_SYS_INFO_CMD 	||
		cmd_code == PAUSE_EXE_CMD		||
		cmd_code == TOGGLE_SAFETY_CMD
		)
			if (!valid_app_status(rxPayload, 1,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
					return lowerlimb_sys_info;

	// Payload size:
	// else if () { }

	////////////////////////////////////////////////////////////////////////////////
	// CHECK IF SYSTEM IS ACTIVE:
	////////////////////////////////////////////////////////////////////////////////

	if (cmd_code == BRAKES_CMD           ||
		cmd_code == AUTO_CALIB_MODE_CMD  ||
		cmd_code == START_RESUME_EXE_CMD ||
		cmd_code == STOP_EXE_CMD		 ||
		cmd_code == PAUSE_EXE_CMD        ||
		cmd_code == TOGGLE_SAFETY_CMD    ||
		cmd_code == SET_CTRLPARAMS       ||
		cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD ||
		cmd_code == SET_TARG_PARAM_ADMCTRL_CMD   ||
		cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD
		)
			if (!valid_app_status(lowerlimb_sys_info.system_state, ON,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_SYSTEM_OFF, ERR_OFFSET))
					return lowerlimb_sys_info;

	////////////////////////////////////////////////////////////////////////////////
	// VALIDATE CONTROL MODES:
	////////////////////////////////////////////////////////////////////////////////

	uint8_t exercise_mode = 0;

	if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD)
		exercise_mode = PassiveTrajectoryCtrl;
	else if (cmd_code == SET_TARG_PARAM_ADMCTRL_CMD)
		exercise_mode = AdmittanceCtrl;
	else if (cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD)
		exercise_mode = ActiveTrajectoryCtrl;

	if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD ||
		cmd_code == SET_TARG_PARAM_ADMCTRL_CMD   ||
		cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD) {
			if (!valid_app_status(lowerlimb_sys_info.activity_state, IDLE,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET))
					return lowerlimb_sys_info;

			if (!valid_app_status(lowerlimb_sys_info.activity_state, CALIB,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET))
					return lowerlimb_sys_info;

			if (lowerlimb_sys_info.activity_state == EXERCISE) {
				if (!valid_app_status(lowerlimb_sys_info.exercise_mode, exercise_mode,
						&lowerlimb_sys_info.app_status, cmd_code, ERR_INVALID_EXERCISE_MODE, ERR_OFFSET))
							return lowerlimb_sys_info;
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	// VALIDATE SYS COMMANDS:
	////////////////////////////////////////////////////////////////////////////////

	if (cmd_code == START_SYS_CMD) { //start system
		// Succeeded to set system ON
		if (!valid_app_status(lowerlimb_sys_info.system_state, OFF,
			&lowerlimb_sys_info.app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
				return lowerlimb_sys_info;
	}

	if (cmd_code == STOP_SYS_CMD) { //stop system
		if (!valid_app_status(lowerlimb_sys_info.system_state, ON,
			&lowerlimb_sys_info.app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
				return lowerlimb_sys_info;
	}

	////////////////////////////////////////////////////////////////////////////////
	// VALIDATE STOP COMMANDS:
	////////////////////////////////////////////////////////////////////////////////

	if (cmd_code == STOP_EXE_CMD ||	cmd_code == SET_CTRLPARAMS)
		if (	!valid_app_status(lowerlimb_sys_info.activity_state, CALIB,
					&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET)
			||	!valid_app_status(lowerlimb_sys_info.activity_state, IDLE,
					&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET)
			||  !valid_app_status(lowerlimb_sys_info.exercise_state, STOPPED,
					&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET) )
				return lowerlimb_sys_info;
	*/

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

		///////////////////////////////////////////////////////////////////////////
		// CMD 01: START_SYS_CMD
		///////////////////////////////////////////////////////////////////////////

		if (cmd_code == START_SYS_CMD) { //start system

			reset_lowerlimb_sys_info();

			lowerlimb_sys_info.system_state   = SYS_ON;
			lowerlimb_sys_info.activity_state = IDLE;
			lowerlimb_sys_info.exercise_state = STOPPED;

			send_OK_resp(cmd_code);

			#if USE_ITM_CMD_CHECK
				printf("   [START_SYS_CMD]: lowerlimb_sys_info.system_state = [%s]\n\n", SYS_STATE_STR[lowerlimb_sys_info.system_state]);
			#endif
		}

		///////////////////////////////////////////////////////////////////////////
		// CMD 02: BRAKES_CMD
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == BRAKES_CMD) {

			#if USE_ITM_CMD_CHECK
				printf("   [ BRAKES_CMD] \n");
				printf("   BEFORE: \n");
				printf("   system_state = [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
				printf("   BRAKES_CMD data: [%d]\n\n", tcpRxData[rx_payload_index]);
			#endif

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

			#if USE_ITM_CMD_CHECK
				printf("   [AUTO_CALIB_MODE_CMD] \n");
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

			///////////////////////////////////////////////////////////////////////////
			// Update activity state to CALIB, otherwise check for errors:
			///////////////////////////////////////////////////////////////////////////

			// Update activity state:
			if (lowerlimb_sys_info.activity_state == IDLE) {
				//enter calibration routines.
				lowerlimb_sys_info.activity_state = CALIB;

				// Activate calibration states:
				*calib_fsens_on = 1;
				*calib_enc_on = 1;

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
			}

			// Check for errors:
			else {
				if (lowerlimb_sys_info.activity_state == EXERCISE) {
					#if USE_ITM_CMD_CHECK
						printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_EXERCISE_ACTIVE]);
					#endif
					send_error_msg(cmd_code, ERR_EXERCISE_ACTIVE);

					lowerlimb_sys_info.app_status = ERR_EXERCISE_ACTIVE + 3;
					return lowerlimb_sys_info;
				}

				else if (lowerlimb_sys_info.activity_state == CALIB) {
					//check if the request is to terminate ongoing calibration
					if (tcpRxData[rx_payload_index] != 255) {
						#if USE_ITM_CMD_CHECK
							printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_CALIB_ACTIVE]);
						#endif
						send_error_msg(cmd_code, ERR_CALIB_ACTIVE);

						lowerlimb_sys_info.app_status = ERR_CALIB_ACTIVE + 3;
						return lowerlimb_sys_info;
					}
				}

				else {
					#if USE_ITM_CMD_CHECK
						printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_GENERAL_NOK]);
					#endif
					send_error_msg(cmd_code, ERR_GENERAL_NOK);

					lowerlimb_sys_info.app_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}
			}

			///////////////////////////////////////////////////////////////////////////
			// Obtain calibration requisites (TODO: delete at a later date):
			///////////////////////////////////////////////////////////////////////////

			/*
			lowerlimb_sys_info.calib_prot_req = tcpRxData[rx_payload_index];

			#if USE_ITM_CMD_CHECK
				if (lowerlimb_sys_info.calib_prot_req < LEN_CALIB_MODES)
					printf("   lowerlimb_sys_info.calib_prot_req: [%s]\n\n", CALIB_MODE_STR[lowerlimb_sys_info.calib_prot_req]);
				else if (lowerlimb_sys_info.calib_prot_req == StopCalib)
					printf("   lowerlimb_sys_info.calib_prot_req: [STOP CALIB]\n\n");
			#endif
			*/

			///////////////////////////////////////////////////////////////////////////
			// Perform calibration per requisites in lowerlimb_sys_info.:
			///////////////////////////////////////////////////////////////////////////

			// if (lowerlimb_sys_info.calib_prot_req == CalibEncodersFS) {
			if (lowerlimb_sys_info.activity_state == CALIB) {

				// Reset Encoders:
				// qei_count_L_reset();
				// qei_count_R_reset();

				// Zero-calibrate force sensor:
				if (*calib_fsens_on) {
					for (int i = 1; i <= 50; i++) {
						force_sensors_read(&hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor,
								&dum_force_end_in_x, &dum_force_end_in_y);

						force_end_in_x_sensor_f += (float) force_end_in_x_sensor * 3.3f / 4095.0f;
						force_end_in_y_sensor_f += (float) force_end_in_y_sensor * 3.3f / 4095.0f;
					}

					force_end_in_x_sensor_f = force_end_in_x_sensor_f / 50.0f;
					force_end_in_y_sensor_f = force_end_in_y_sensor_f / 50.0f;

					set_force_sensor_zero_offset(force_end_in_x_sensor_f, force_end_in_y_sensor_f);
					*calib_fsens_on = 0;

					#if USE_ITM_CMD_CHECK
						printf("   lowerlimb_app_state_tcpip(): [Force sensors calibrated] \n\n");
					#endif
				}

				// If all calibrations are completed, update activity state (NOTE: *calib_enc_on is only zero'd by test_real_time()):
				else if (*calib_fsens_on == 0 && *calib_enc_on == 0) {
					#if USE_ITM_CMD_CHECK
						printf("   lowerlimb_app_state_tcpip(): [Encoders calibrated] \n\n");
					#endif

					// Send calibration response:
					send_calibration_resp(lowerlimb_sys_info.calib_prot_req, 100, 2);

					// Reset activity to IDLE:
					// set_activity_idle(); // possible bug!
					lowerlimb_sys_info.activity_state = IDLE;

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
			}
		}

		///////////////////////////////////////////////////////////////////////////
		// CMD 04: START_RESUME_EXE_CMD
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == START_RESUME_EXE_CMD) {

			#if USE_ITM_CMD_CHECK
				printf("   [START_RESUME_EXE_CMD]: \n");
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

			//reset emergency alerts if any
			reset_emergency_alerts();

			// If exercise_state in paused state, resume exercise (TODO: REVISE)
			// If activity is in IDLE state, start exercise to SETUP state (TODO: REVISE)
			// Rest are errors (TODO: REVISE)

			if (lowerlimb_sys_info.activity_state == IDLE) {
				// Check if device has been calibrated:
				if (!valid_app_status(lowerlimb_sys_info.isCalibrated, NO,
					&lowerlimb_sys_info.app_status, cmd_code, ERR_CALIBRATION_NEEDED, ERR_OFFSET))
						return lowerlimb_sys_info;

				// Check exercise mode:
				lowerlimb_sys_info.exercise_mode = (exercise_mode_t)tcpRxData[rx_payload_index];
				rx_payload_index += 1;

				if (lowerlimb_sys_info.exercise_mode == PassiveTrajectoryCtrl) { }
				else if (lowerlimb_sys_info.exercise_mode == AdmittanceCtrl) { }
				else if (lowerlimb_sys_info.exercise_mode == ActiveTrajectoryCtrl) { }
				else {
					#if USE_ITM_CMD_CHECK
						printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_INVALID_EXERCISE_MODE]);
					#endif
					send_error_msg(cmd_code, ERR_INVALID_EXERCISE_MODE);

					lowerlimb_sys_info.app_status = ERR_INVALID_EXERCISE_MODE + 3;
					return lowerlimb_sys_info;
				}

				// Clear motor settings since each start exercise is considered a fresh start:
				clear_lowerlimb_motors_settings(LL_motors_settings);

				// Clear transition mode params to remove any last exercise's parameters:
				clear_transition_mode_params();

				rx_payload_index += 1;

				/*
				if (tcpRxData[rx_payload_index] == 0x01) {
					lowerlimb_brakes_command.l_brake_disengage = true;
					lowerlimb_brakes_command.r_brake_disengage = true;
				}
				else {
					lowerlimb_brakes_command.l_brake_disengage = false;
					lowerlimb_brakes_command.r_brake_disengage = false;
				}
				*/

				// set activity to exercise
				lowerlimb_sys_info.activity_state = EXERCISE;

				// exercise state goes to SETUP:
				lowerlimb_sys_info.exercise_state = SETUP; // can we switch to RUNNING directly?
			}
			else if (lowerlimb_sys_info.activity_state == EXERCISE) {
				if (!valid_app_status(lowerlimb_sys_info.exercise_state, PAUSED,
					&lowerlimb_sys_info.app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
						return lowerlimb_sys_info;

				// if NOT paused, set to running
				lowerlimb_sys_info.exercise_state = RUNNING;
			}
			else if (lowerlimb_sys_info.activity_state == CALIB) {
				// send active calibration error message:
				#if USE_ITM_CMD_CHECK
					printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_CALIB_ACTIVE]);
				#endif
				send_error_msg(cmd_code, ERR_CALIB_ACTIVE);

				lowerlimb_sys_info.app_status = ERR_CALIB_ACTIVE + 3;
				return lowerlimb_sys_info;
			}
			else {
				// cover the rest with general NOK:
				#if USE_ITM_CMD_CHECK
					printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_GENERAL_NOK]);
				#endif
				send_error_msg(cmd_code, ERR_GENERAL_NOK);

				lowerlimb_sys_info.app_status = ERR_GENERAL_NOK + 3;
				return lowerlimb_sys_info;
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
		// CMD 05: STOP_EXE_CMD
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == STOP_EXE_CMD) { //stop exercise

			#if USE_ITM_CMD_CHECK
				printf("   [USE_ITM_CMD_CHECK]: \n");
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

			if (TRAJ_PARAMS_VARIABLE_ON) {
				stop_exe_cmd_count++;

				if (stop_exe_cmd_count == 1)
					lowerlimb_sys_info.exercise_state = SLOWING;
				else if (stop_exe_cmd_count == 2) {
					lowerlimb_sys_info.activity_state = IDLE;
					lowerlimb_sys_info.exercise_state = STOPPED;

					stop_exe_cmd_count = 0;
				}
			}
			else {
				lowerlimb_sys_info.activity_state = IDLE;
				lowerlimb_sys_info.exercise_state = STOPPED;
			}

			// reset:
			clear_lowerlimb_motors_settings(LL_motors_settings);

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
		// CMD 06: STOP_SYS_CMD
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == STOP_SYS_CMD) { //stop system
			lowerlimb_sys_info.system_state   = SYS_OFF;
			lowerlimb_sys_info.activity_state = IDLE;
			lowerlimb_sys_info.exercise_state = STOPPED;

			//Reset brake command
			lowerlimb_brakes_command.l_brake_disengage = false;
			lowerlimb_brakes_command.r_brake_disengage = false;

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == NO_CMD) {	}

		else {
			// tx unknown command error:
			send_error_msg(cmd_code, ERR_UNKNOWN);
			lowerlimb_sys_info.app_status = ERR_UNKNOWN + 3;
			return lowerlimb_sys_info;
		} // end (if cmd_code == )

		cmd_code_prev = cmd_code;

	return lowerlimb_sys_info;
}
