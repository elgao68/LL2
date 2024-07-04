/////////////////////////////////////////////////////////////////////////////
//
//  lowerlimb_app_scripts_template.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "lowerlimb_app.h"

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// TCP/IP APP STATE - GAO
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

lowerlimb_sys_info_t
lowerlimb_app_state_template(uint8_t ui8EBtnState, uint8_t ui8Alert, traj_ctrl_params_t* traj_ctrl_params,
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

	// Counters:
	static int step_i = 0;

	//reset
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

	if (isTCPConnected() == 0) //disconnected
			{
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

		if (tmp_chksum != tcpRxData[checkSum_index])  //failed comparison
				{
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
				printf("step_i [%d]: cmd = [%s]\n", step_i, CMD_STR[cmd_code]);
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

		/*
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
		*/

	    //////////////////////////////////////////////////////////////////////////////////////
		// VALIDATE CONTROL MODES:
		//////////////////////////////////////////////////////////////////////////////////////

		uint8_t exercise_mode = 0;

		/*
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
		*/

	    //////////////////////////////////////////////////////////////////////////////////////
		// VALIDATE SYS COMMANDS:
	    //////////////////////////////////////////////////////////////////////////////////////

		/*
		if (cmd_code ==  START_SYS_CMD) { //start system
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
		if (cmd_code == STOP_EXE_CMD ||	cmd_code == SET_CTRLPARAMS) {
			if (!valid_app_status(lowerlimb_sys_info.activity_state, CALIB,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET))
					return lowerlimb_sys_info;
			else if (!valid_app_status(lowerlimb_sys_info.activity_state, IDLE,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET))
					return lowerlimb_sys_info;

			if (!valid_app_status(lowerlimb_sys_info.exercise_state, STOPPED,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET))
					return lowerlimb_sys_info;
		}
		*/

	    //////////////////////////////////////////////////////////////////////////////////////
		// EXECUTE COMMAND:
	    //////////////////////////////////////////////////////////////////////////////////////

		if (cmd_code == SET_UNIX_CMD) { //set unix

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == START_SYS_CMD) { //start system

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == STOP_SYS_CMD) { //stop system

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == RESET_SYS_CMD) { //restart system

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == READ_DEV_ID_CMD) { //read device ID

			send_resp_msg(cmd_code, lowerlimb_sys_info.device_id, sizeof(lowerlimb_sys_info.device_id));
		}

		else if (cmd_code == READ_SYS_INFO_CMD) { //read system info

			send_lowerlimb_sys_info(&lowerlimb_sys_info, tmp_resp_msg, cmd_code);
		}

		else if (cmd_code == AUTO_CALIB_MODE_CMD) { //enter calibration

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == PAUSE_EXE_CMD) { //pause exercise

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == STOP_EXE_CMD) { //stop exercise

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == TOGGLE_SAFETY_CMD) { //enable/disable safety features

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == BRAKES_CMD) {

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == START_RESUME_EXE_CMD) { //start/resume exercise ; rxPayload == 11

			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET CONTROL GAINS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_CTRLPARAMS) {

			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET PASSIVE TRAJECTORY CONTROL PARAMS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD) {

			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET ADMITTANCE CONTROL PARAMS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_ADMCTRL_CMD) { // rxPayload 65

			send_OK_resp(cmd_code);
		}

		//////////////////////////////////////////////////////////////////////////////////////
		// SET ACTIVE TRAJECTORY CONTROL PARAMS:
		//////////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD) {

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
