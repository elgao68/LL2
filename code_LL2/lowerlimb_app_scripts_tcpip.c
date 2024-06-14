/////////////////////////////////////////////////////////////////////////////
//
//  lowerlimb_app_scripts_tcpip.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "lowerlimb_app.h"

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// TCP/IP APP STATE - GAO - TEMPLATE:
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

lowerlimb_sys_info_t
lowerlimb_app_state_tcpip(uint8_t ui8EBtnState, uint8_t ui8Alert, traj_ctrl_params_t* traj_ctrl_params,
		admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings, uint16_t* cmd_code_copy) {

	uint16_t n_cnt = 0;
	uint8_t tmp_chksum = 0;
	//uint8_t tmp_index = 0;
	uint8_t tmp_chk = 0;
	uint16_t cmd_code = 0;
	uint64_t ui64_tmp = 0;
	uint8_t rxPayload = 0;
	uint8_t tmp_resp_msg[465];
	//int16_t i6TmpQEIL, i6TmpQEIR;

	//Constants & Variables for parsing index
	const uint8_t startByte0_index = 0;
	const uint8_t startByte1_index = 1;
	const uint8_t msgId_index = 2;
	const uint8_t cmdCode_index = 3;
	const uint8_t payloadLen_index = 5;
	const uint8_t payloadStart_index = 7;
	uint8_t checkSum_index;
	uint8_t endByte0_index;
	uint8_t endByte1_index;

	uint8_t rx_payload_index = payloadStart_index;
	// uint16_t resp_payload_index = 0;
	//const uint8_t exercise_payload_length = 89;

	/*
	//Configurable Control Parameters
	float fbgain;
	float ffgain;
	float compgain;

	//Trajectory Parameters
	float cycle_period;
	float exp_blending_time;
	bool cycle_dir;
	float semiaxis_x;
	float semiaxis_y;
	float rot_angle;

	//Dynamic Parameters
	float inertia_x;
	float inertia_y;
	float damping;
	float stiffness;
	float p_eq_x;
	float p_eq_y;

	//Force parameters
	float F_assist_resist;
	float Fx_offset;
	float Fy_offset;
	*/

	//Force Sensor
	uint32_t force_end_in_x_sensor = 0;
	uint32_t force_end_in_y_sensor = 0;
	uint32_t dum_force_end_in_x = 0;
	uint32_t dum_force_end_in_y = 0;
	float force_end_in_x_sensor_f = 0;
	float force_end_in_y_sensor_f = 0;

	// Counters:
	static int step_i = 0;

	// Auxiliary variables:
	#if USE_ITM_CMD_CHECK
		uint8_t idx_exerc_mode  = lowerlimb_sys_info.exercise_mode;

		uint8_t idx_sys_state   = lowerlimb_sys_info.system_state;
		uint8_t idx_activ_state = lowerlimb_sys_info.activity_state;
		uint8_t idx_exerc_state = lowerlimb_sys_info.exercise_state;
	#endif

	static uint8_t stop_exe_cmd_count = 0; // stop command counter - this is used to accommodate the SLOWING case

	// Reset:
	lowerlimb_sys_info.app_status = 0;
	lowerlimb_sys_info.calib_prot_req = 0;

	////////////////////////////////////
	// update system operation status
	////////////////////////////////////

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

	////////////////////////////////////
	//Update local Unix time
	////////////////////////////////////

	update_unix_time();

	////////////////////////////////////
	// Check if TCP connected
	////////////////////////////////////

	if (isTCPConnected() == 0) { //disconnected
		//set activity to IDLE
		set_activity_idle();

		//set exercise_state to STOPPED
		stop_exercise(LL_motors_settings);

		//set system state to OFF
		set_system_off();

		return lowerlimb_sys_info;
	}

	////////////////////////////////////
	// Parsing and executing TCP messages
	////////////////////////////////////

	//check if there's new message
	if (ethernet_w5500_new_rcv_data()) {

		/////////////////////////////////
		//get tcp data
		/////////////////////////////////

		memset(tcpRxData, 0, sizeof(tcpRxData));
		tcpRxLen = ethernet_w5500_rcv_data(tcpRxData);

		checkSum_index = tcpRxLen - 4;
		endByte0_index = tcpRxLen - 3;
		endByte1_index = tcpRxLen - 2;

		//check for preample and postample
		if ((tcpRxData[startByte0_index] != PREAMP_TCP[0])
				|| (tcpRxData[startByte1_index] != PREAMP_TCP[1])
				|| (tcpRxData[endByte0_index] != POSTAMP_TCP[0])
				|| (tcpRxData[endByte1_index] != POSTAMP_TCP[1])) {
			lowerlimb_sys_info.app_status = 1;
			return lowerlimb_sys_info;
		}

		/////////////////////////////////
		//error checksum comparison
		/////////////////////////////////

		tmp_chksum = 0;
		for (n_cnt = 2; n_cnt < checkSum_index; n_cnt++) {
			tmp_chksum ^= tcpRxData[n_cnt];
		}

		if (tmp_chksum != tcpRxData[checkSum_index]) { //failed comparison
			send_error_msg(cmd_code, ERR_CHECKSUM_FAILED);
			lowerlimb_sys_info.app_status = ERR_CHECKSUM_FAILED + 3;
			return lowerlimb_sys_info;
		}

		/////////////////////////////////
		//check if command message. Rest is discarded
		/////////////////////////////////

		if (tcpRxData[msgId_index] != CMD_MSG_TYPE) {
			lowerlimb_sys_info.app_status = 2;
			return lowerlimb_sys_info;
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// Parse command code:
		//////////////////////////////////////////////////////////////////////////////////////

		cmd_code = ((uint16_t) tcpRxData[cmdCode_index] << 8)
				+ (uint16_t) tcpRxData[cmdCode_index + 1];

		*cmd_code_copy = cmd_code;

		rxPayload = ((uint16_t) tcpRxData[payloadLen_index] << 8)
				+ (uint16_t) tcpRxData[payloadLen_index + 1];

		//rxPayload = tcpRxData[5];

		//////////////////////////////////////////////////////////////////////////////////////
		// Console output:
		//////////////////////////////////////////////////////////////////////////////////////

		#if USE_ITM_CMD_CHECK
			static int cmd_code_prev = 0;

			if (cmd_code != cmd_code_prev) {
				printf("\n");
				printf("step_i [%d]: cmd = [%s]\n\n", step_i, CMD_STR[cmd_code]);
				cmd_code_prev = cmd_code;
			}
			step_i++;
		#endif

	    //////////////////////////////////////////////////////////////////////////////////////
		// VALIDATE PAYLOAD SIZES:
	    //////////////////////////////////////////////////////////////////////////////////////

		// Payload size: 1
		if (cmd_code == START_SYS_CMD		||
			cmd_code == STOP_SYS_CMD		||
			cmd_code == RESET_SYS_CMD		||
			cmd_code == READ_DEV_ID_CMD		||
			cmd_code == READ_SYS_INFO_CMD	||
			cmd_code == AUTO_CALIB_MODE_CMD ||
			cmd_code == PAUSE_EXE_CMD		||
			cmd_code == STOP_EXE_CMD		||
			cmd_code == TOGGLE_SAFETY_CMD	||
			cmd_code == BRAKES_CMD)
				if (!valid_app_status(rxPayload, 1,
					&lowerlimb_sys_info.app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
						return lowerlimb_sys_info;

		// Payload size:

	    //////////////////////////////////////////////////////////////////////////////////////
		// CHECK IF SYSTEM IS ACTIVE:
	    //////////////////////////////////////////////////////////////////////////////////////

		if (cmd_code == AUTO_CALIB_MODE_CMD ||
			cmd_code == PAUSE_EXE_CMD ||
			cmd_code == STOP_EXE_CMD ||
			cmd_code == TOGGLE_SAFETY_CMD ||
			cmd_code == BRAKES_CMD ||
			cmd_code == START_RESUME_EXE_CMD ||
			cmd_code == SET_CTRLPARAMS ||
			cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD ||
			cmd_code == SET_TARG_PARAM_ADMCTRL_CMD ||
			cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD)
				if (!valid_app_status(lowerlimb_sys_info.system_state, ON,
					&lowerlimb_sys_info.app_status, cmd_code, ERR_SYSTEM_OFF, ERR_OFFSET))
						return lowerlimb_sys_info;

	    //////////////////////////////////////////////////////////////////////////////////////
		// VALIDATE CONTROL MODES:
		//////////////////////////////////////////////////////////////////////////////////////

		uint8_t exercise_mode = 0;

		if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD)
			exercise_mode = PassiveTrajectoryCtrl;
		else if (cmd_code == SET_TARG_PARAM_ADMCTRL_CMD)
			exercise_mode = AdmittanceCtrl;
		else if (cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD)
			exercise_mode = ActiveTrajectoryCtrl;

		if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD ||
			cmd_code == SET_TARG_PARAM_ADMCTRL_CMD ||
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

	    //////////////////////////////////////////////////////////////////////////////////////
		// VALIDATE SYS COMMANDS:
	    //////////////////////////////////////////////////////////////////////////////////////

		/*
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
		*/

	    //////////////////////////////////////////////////////////////////////////////////////
		// VALIDATE STOP COMMANDS:
	    //////////////////////////////////////////////////////////////////////////////////////

		/*
		if (cmd_code == STOP_EXE_CMD ||	cmd_code == SET_CTRLPARAMS)
			if (	!valid_app_status(lowerlimb_sys_info.activity_state, CALIB,
						&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET)
				||	!valid_app_status(lowerlimb_sys_info.activity_state, IDLE,
						&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET)
				||  !valid_app_status(lowerlimb_sys_info.exercise_state, STOPPED,
						&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET) )
					return lowerlimb_sys_info;
		*/

	    //////////////////////////////////////////////////////////////////////////////////////
		// EXECUTE COMMAND:
	    //////////////////////////////////////////////////////////////////////////////////////

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

			lowerlimb_sys_info.system_state = ON;
			lowerlimb_sys_info.activity_state = IDLE;
			lowerlimb_sys_info.exercise_state = STOPPED;

			send_OK_resp(cmd_code);
		}

		///////////////////////////////////////////////////////////////////////////
		// CMD 02: BRAKES_CMD
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == BRAKES_CMD) {
			#if USE_ITM_CMD_CHECK
				printf("   BRAKES_CMD data: [%d]\n", tcpRxData[rx_payload_index]);
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
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

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
			else if (lowerlimb_sys_info.activity_state == IDLE) {
				//enter calibration routines.
				lowerlimb_sys_info.activity_state = CALIB;
			}
			else {
				#if USE_ITM_CMD_CHECK
					printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_GENERAL_NOK]);
				#endif
				send_error_msg(cmd_code, ERR_GENERAL_NOK);

				lowerlimb_sys_info.app_status = ERR_GENERAL_NOK + 3;
				return lowerlimb_sys_info;
			}

			lowerlimb_sys_info.calib_prot_req = tcpRxData[rx_payload_index];

			uint8_t calib_prot_req = lowerlimb_sys_info.calib_prot_req;

			#if USE_ITM_CMD_CHECK
				if (calib_prot_req < LEN_CALIB_MODES)
					printf("   calib_prot_req: [%s]\n\n", CALIB_MODE_STR[calib_prot_req]);
				else if (calib_prot_req == StopCalib)
					printf("   calib_prot_req: [STOP CALIB]\n\n");
			#endif

			if (calib_prot_req == CalibEncodersFS) { // Calibrate Both Motors
				//Reset Encoder
				qei_count_L_reset();
				qei_count_R_reset();

				// Zero calib Force Sensor
				for (int i = 1; i <= 50; i++) {
					force_sensors_read(&hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor,
							&dum_force_end_in_x, &dum_force_end_in_y);

					force_end_in_x_sensor_f += (float) force_end_in_x_sensor * 3.3f / 4095.0f;
					force_end_in_y_sensor_f += (float) force_end_in_y_sensor * 3.3f / 4095.0f;
				}

				force_end_in_x_sensor_f = force_end_in_x_sensor_f / 50.0f;
				force_end_in_y_sensor_f = force_end_in_y_sensor_f / 50.0f;

				set_force_sensor_zero_offset(force_end_in_x_sensor_f, force_end_in_y_sensor_f);

				//send resp
				send_calibration_resp(lowerlimb_sys_info.calib_prot_req, 100, 2);

				// reset activity to IDLE:
				// set_activity_idle(); // possible bug!
				lowerlimb_sys_info.activity_state = IDLE;
			}
			/*
			else if (calib_prot_req == CalibEncoders) {	//Calibrate Left Motor
				qei_count_L_reset();
				qei_count_R_reset();

				//send resp
				send_calibration_resp(lowerlimb_sys_info.calib_prot_req, 100, 2);

				//reset acitivity to IDLE
				set_activity_idle();
			}
			else if (calib_prot_req == CalibForceSensor) {	//Calibrate Right Motor
				for (int i = 1; i <= 50; i++) {
					force_sensors_read(&hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor,
									 &dum_force_end_in_x, &dum_force_end_in_y);
					force_end_in_x_sensor_f += (float) force_end_in_x_sensor * 3.3f / 4095.0f;
					force_end_in_y_sensor_f += (float) force_end_in_y_sensor * 3.3f / 4095.0f;
				}

				force_end_in_x_sensor_f = force_end_in_x_sensor_f / 50.0f;
				force_end_in_y_sensor_f = force_end_in_y_sensor_f / 50.0f;

				set_force_sensor_zero_offset(force_end_in_x_sensor_f, force_end_in_y_sensor_f);

				//send resp
				send_calibration_resp(lowerlimb_sys_info.calib_prot_req, 100, 2);

				//reset acitivity to IDLE
				set_activity_idle();
			}
			else if (calib_prot_req == AutoCalibEncodersFS) {	//Auto Calibrate Both Axis
				//To be implemented later
				send_error_msg(cmd_code, ERR_GENERAL_NOK);
			}

			else if (calib_prot_req == StopCalib) { // Terminate Calibration
				// send response
				send_calibration_resp(lowerlimb_sys_info.calib_prot_req, 0, 0);

				// go back to idle
				set_activity_idle();
			}
			else
				send_error_msg(cmd_code, ERR_GENERAL_NOK);
			*/

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
		// CMD 04: START_RESUME_EXE_CMD
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == START_RESUME_EXE_CMD) {

			#if USE_ITM_CMD_CHECK
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

			// start/resume exercise ; rxPayload == 11

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

				// if (lowerlimb_sys_info.exercise_mode == ImpedanceCtrl) {
				// }
				if (lowerlimb_sys_info.exercise_mode == PassiveTrajectoryCtrl) {
				}
				else if (lowerlimb_sys_info.exercise_mode == AdmittanceCtrl) {
				}
				else if (lowerlimb_sys_info.exercise_mode == ActiveTrajectoryCtrl) {
				}
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
				printf("   BEFORE: \n");
				printf("   system_state:   [%s]\n",   SYS_STATE_STR[idx_sys_state]  );
				printf("   activity_state: [%s]\n", ACTIV_STATE_STR[idx_activ_state]);
				printf("   exercise_state: [%s]\n", EXERC_STATE_STR[idx_exerc_state]);
				printf("\n");
			#endif

			if (USE_TRAJ_PARAMS_VARIABLE) {
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
			lowerlimb_sys_info.system_state   = OFF;
			lowerlimb_sys_info.activity_state = IDLE;
			lowerlimb_sys_info.exercise_state = STOPPED;

			//Reset brake command
			lowerlimb_brakes_command.l_brake_disengage = false;
			lowerlimb_brakes_command.r_brake_disengage = false;

			send_OK_resp(cmd_code);
		}

		///////////////////////////////////////////////////////////////////////////
		// CMD OTHER:
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_UNIX_CMD) { //set unix
			/*
			//pack UNIX
			ui64_tmp = 0;
			memcpy_msb(&ui64_tmp, &tcpRxData[payloadStart_index], sizeof(lowerlimb_sys_info.unix));

			//check if UNIX is valid
			if (!valid_app_status(set_unix_time(ui64_tmp), 0,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_INVALID_UNIX, ERR_OFFSET))
					return lowerlimb_sys_info;
			*/
			send_OK_resp(cmd_code);
		}

		else if (cmd_code == RESET_SYS_CMD) { //restart system
			/*
			HAL_Delay(100); //wait for msg to be transmitted
			HAL_NVIC_SystemReset(); //reset MCU
			*/
			send_OK_resp(cmd_code);
		}

		else if (cmd_code == READ_DEV_ID_CMD) { //read device ID
			// send resp message with device ID
			send_resp_msg(cmd_code, lowerlimb_sys_info.device_id, sizeof(lowerlimb_sys_info.device_id));
		}

		else if (cmd_code == READ_SYS_INFO_CMD) { //read system info
			// pack data
			send_lowerlimb_sys_info(&lowerlimb_sys_info, tmp_resp_msg, cmd_code);
		}

		else if (cmd_code == PAUSE_EXE_CMD) { //pause exercise
			/*
			// only can paused if exercise_state is in RUNNING mode:
			if (lowerlimb_sys_info.activity_state == EXERCISE &&
				lowerlimb_sys_info.exercise_state == RUNNING) {
					lowerlimb_sys_info.exercise_state = PAUSED;
					// send OK resp
					send_OK_resp(cmd_code);
			}
			else {
				//return error message
				send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
				lowerlimb_sys_info.app_status = ERR_EXERCISE_NOT_RUNNING + 3;
				return lowerlimb_sys_info;
			}
			*/
		}

		else if (cmd_code == TOGGLE_SAFETY_CMD) { //enable/disable safety features
			/*
			lowerlimb_sys_info.safetyOFF = tcpRxData[rx_payload_index];
			*/
			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET CONTROL GAINS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_CTRLPARAMS) {
			/*
			// Assign control parameter values:
			memcpy_msb(&fbgain, &tcpRxData[rx_payload_index], sizeof(fbgain));
			rx_payload_index += sizeof(fbgain);
			memcpy_msb(&ffgain, &tcpRxData[rx_payload_index], sizeof(ffgain));
			rx_payload_index += sizeof(ffgain);
			memcpy_msb(&compgain, &tcpRxData[rx_payload_index], sizeof(compgain));

			//Set control parameters
			configure_ctrl_params(fbgain, ffgain, compgain);
			*/
			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET PASSIVE TRAJECTORY CONTROL PARAMS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD) {
			/*
			// Assign control parameter values:
			memcpy_msb(&cycle_period, &tcpRxData[rx_payload_index],
					   sizeof(cycle_period));
			rx_payload_index += sizeof(cycle_period);
			memcpy_msb(&exp_blending_time, &tcpRxData[rx_payload_index],
					   sizeof(exp_blending_time));
			rx_payload_index += sizeof(exp_blending_time);
			memcpy_msb(&semiaxis_x, &tcpRxData[rx_payload_index],
					   sizeof(semiaxis_x));
			rx_payload_index += sizeof(semiaxis_x);
			memcpy_msb(&semiaxis_y, &tcpRxData[rx_payload_index],
					   sizeof(semiaxis_y));
			rx_payload_index += sizeof(semiaxis_y);
			memcpy_msb(&rot_angle, &tcpRxData[rx_payload_index], sizeof(rot_angle));
			rx_payload_index += sizeof(rot_angle);
			cycle_dir = tcpRxData[rx_payload_index];

			#if SET_CTRL_PARAMETERS
				//Set passive trajectory control parameters:
				*traj_ctrl_params = set_traj_ctrl_params(cycle_period, exp_blending_time,
						semiaxis_x, semiaxis_y, rot_angle, cycle_dir);
			#endif
			lowerlimb_sys_info.exercise_state = RUNNING;
			*/
			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET ADMITTANCE CONTROL PARAMS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_ADMCTRL_CMD) { // rxPayload 65
			/*
			// Assign control parameter values:
			memcpy_msb(&inertia_x, &tcpRxData[rx_payload_index], sizeof(inertia_x));
			rx_payload_index += sizeof(inertia_x);
			memcpy_msb(&inertia_y, &tcpRxData[rx_payload_index], sizeof(inertia_y));
			rx_payload_index += sizeof(inertia_y);
			memcpy_msb(&damping, &tcpRxData[rx_payload_index], sizeof(damping));
			rx_payload_index += sizeof(damping);
			memcpy_msb(&stiffness, &tcpRxData[rx_payload_index], sizeof(stiffness));
			rx_payload_index += sizeof(stiffness);
			memcpy_msb(&p_eq_x, &tcpRxData[rx_payload_index], sizeof(p_eq_x));
			rx_payload_index += sizeof(p_eq_x);
			memcpy_msb(&p_eq_y, &tcpRxData[rx_payload_index], sizeof(p_eq_y));
			rx_payload_index += sizeof(p_eq_y);
			memcpy_msb(&Fx_offset, &tcpRxData[rx_payload_index], sizeof(Fx_offset));
			rx_payload_index += sizeof(Fx_offset);
			memcpy_msb(&Fy_offset, &tcpRxData[rx_payload_index], sizeof(Fy_offset));
			rx_payload_index += sizeof(Fy_offset);

			#if SET_CTRL_PARAMETERS
				//Set admittance control params:
				*admitt_model_params = set_admitt_model_params(inertia_x, inertia_y, damping,
						stiffness, p_eq_x, p_eq_y, Fx_offset, Fy_offset);
			#endif
			lowerlimb_sys_info.exercise_state = RUNNING;
			*/
			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET ACTIVE TRAJECTORY CONTROL PARAMS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD) {
			/*
			// Assign control parameter values:
			memcpy_msb(&cycle_period, &tcpRxData[rx_payload_index],
					   sizeof(cycle_period));
			rx_payload_index += sizeof(cycle_period);
			memcpy_msb(&exp_blending_time, &tcpRxData[rx_payload_index],
					   sizeof(exp_blending_time));
			rx_payload_index += sizeof(exp_blending_time);
			memcpy_msb(&inertia_x, &tcpRxData[rx_payload_index], sizeof(inertia_x));
			rx_payload_index += sizeof(inertia_x);
			memcpy_msb(&inertia_y, &tcpRxData[rx_payload_index], sizeof(inertia_y));
			rx_payload_index += sizeof(inertia_y);
			memcpy_msb(&damping, &tcpRxData[rx_payload_index], sizeof(damping));
			rx_payload_index += sizeof(damping);
			memcpy_msb(&stiffness, &tcpRxData[rx_payload_index], sizeof(stiffness));
			rx_payload_index += sizeof(stiffness);
			memcpy_msb(&F_assist_resist, &tcpRxData[rx_payload_index],
					   sizeof(F_assist_resist));
			rx_payload_index += sizeof(F_assist_resist);

			#if SET_CTRL_PARAMETERS
				//Set passive trajectory control parameters:
				*traj_ctrl_params = set_traj_ctrl_params(cycle_period, exp_blending_time,
						semiaxis_x, semiaxis_y, rot_angle, cycle_dir);

				//Set admittance control params:
				*admitt_model_params = set_admitt_model_params(inertia_x, inertia_y, damping,
						stiffness, p_eq_x, p_eq_y, Fx_offset, Fy_offset);
			#endif

			lowerlimb_sys_info.exercise_state = RUNNING;
			*/
			send_OK_resp(cmd_code);
		}

		else if (cmd_code == NO_CMD) {
		}

		else {
			// tx unknown command error:
			send_error_msg(cmd_code, ERR_UNKNOWN);
			lowerlimb_sys_info.app_status = ERR_UNKNOWN + 3;
			return lowerlimb_sys_info;
		} // end (if cmd_code == )
	} // if (ethernet_w5500_new_rcv_data())

	return lowerlimb_sys_info;
}
