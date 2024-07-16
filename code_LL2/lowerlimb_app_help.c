/* ========================================
 *
 * Copyright Thesis Pte Ltd, 2018
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Thesis Pte Ltd.
 *
 * Version: 1.5
 * Updated: 9 Oct 2018
 *
 * ========================================
 */
#include <lowerlimb_app.h>
#include <stdlib.h>
#include <string.h>
#include "w5500_app.h"
#include "timer.h"

#if (MOTOR_DRIVER_OPS)
	#include <qei_motor_drivers_LL2.h>
#endif

///////////////////////////////////////////////////////////////////////
// Flags for is_valid_cmd_code_tcp_app() - CRITICAL:
///////////////////////////////////////////////////////////////////////

#define TEST_CMD_CODE_TCP_APP_PAYLOAD 			1
#define TEST_CMD_CODE_TCP_APP_SYS_ON 			0
#define TEST_CMD_CODE_TCP_APP_CTRL_MODE 		1
#define TEST_CMD_CODE_TCP_APP_START_STOP_SYS	0
#define TEST_CMD_CODE_TCP_APP_STOP_EXE			0

///////////////////////////////////////////////////////////////////////
// Serial number:
///////////////////////////////////////////////////////////////////////

// Format PP-VVVRYYXXX-A | 11 bytes reserved for future
// PP – Product Initials i.e. HM
// VVV – Product Version i.e. v10 for v1.0
// R – R for Research, M for Medical/Commercial Version
// YY – Year of manufacturing i.e. 20 for 2020
// XXX – Serial number for product version
// A – Alphabets from A to Z, where each alphabet represent modification.
// E.g. HM-V10M20001-A

const uint8_t DEVICE_SERIAL_ID[25] = { 'D', 'C', '-', 'V', '1', '0', 'M', '2',
		'2', '0', '0', '1', '-', 'A' };

///////////////////////////////////////////////////////////////////////
// Lower-limb robot system info:
///////////////////////////////////////////////////////////////////////

// TODO: check whether this declaration is needed (2024.07.10):
// extern lowerlimb_sys_info_t lowerlimb_sys_info;

///////////////////////////////////////////////////////////////////////
// State initialization function:
///////////////////////////////////////////////////////////////////////
/**
 @brief: Start in the APP.
 @retval: 0 = success, 1 = failed
 */
uint8_t lowerlimb_app_state_initialize(uint64_t init_unix, uint8_t maj_ver,
		uint8_t min_ver, uint8_t patch_ver, lowerlimb_motors_settings_t* LL_motors_settings) {
	//Zero information
	reset_lowerlimb_sys_info();
	if (stop_exercise(LL_motors_settings) != 0)
		return 1;

	//set sys info
	lowerlimb_sys_info.unix = init_unix;
	lowerlimb_sys_info.fw_maj_ver = maj_ver;
	lowerlimb_sys_info.fw_min_ver = min_ver;
	lowerlimb_sys_info.fw_patch_ver = patch_ver;

	lowerlimb_brakes_command.l_brake_disengage = false;
	lowerlimb_brakes_command.r_brake_disengage = false;

	return 0;
}

///////////////////////////////////////////////////////////////////////
// Message validation functions, high-level:
///////////////////////////////////////////////////////////////////////

uint8_t
is_valid_rcv_data_cmd_code(uint16_t* cmd_code_ref, uint8_t ui8EBtnState, uint8_t ui8Alert,
		uint8_t use_software_msg_list, lowerlimb_sys_info_t* lowerlimb_sys, uint8_t tcp_rx[]) {

	static uint16_t cmd_code             = 0;      // CRITICAL to make it static
	static exercise_mode_t exercise_mode = NoCtrl; // CRITICAL to make it static

	uint8_t rxPayload        = 0;

	////////////////////////////////////
	// Test variables:
	////////////////////////////////////

	uint8_t is_valid_msg_test = 0; // NOTE: this variable gets reused for several tests

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

	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	// Check if TCP connected:
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	// NOTE: an ITM console output here will give continuous messages:
	if (isTCPConnected() == 0)
		return 0;

	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	// Validate TCP messages:
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////
	// Check if there is a valid message:
	////////////////////////////////////////////////////////////////////////////////

	if (ethernet_w5500_new_rcv_data()) {
		memset(tcp_rx, 0, sizeof(tcp_rx)); // CRITICAL: tcp_rx references a global variable
		is_valid_msg_test = is_valid_w5500_msg(tcp_rx);
	}
	else
		return 0;

	////////////////////////////////////////////////////////////////////////////////
	// Parse command code & payload:
	////////////////////////////////////////////////////////////////////////////////

	if (is_valid_msg_test) {
		cmd_code =  ((uint16_t) tcp_rx[cmdCode_index] << 8) +
					      (uint16_t) tcp_rx[cmdCode_index + 1];

		rxPayload = ((uint16_t) tcp_rx[payloadLen_index] << 8) +
					 (uint16_t) tcp_rx[payloadLen_index + 1];
	}
	else {
		#if USE_ITM_VALID_CMD_CHECK
			printf("   <<is_valid_rcv_data_cmd_code()>> s_valid_w5500_msg() FAILED \n\n");
		#endif

		return 0;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Validate command code (NOTE: 'is_valid' functions reject a zero cmd_code (NO_CMD / NO_MSG_TCP) as invalid):
	////////////////////////////////////////////////////////////////////////////////

	if (use_software_msg_list)
		is_valid_msg_test = is_valid_payload_size_software(cmd_code, rxPayload,
				&lowerlimb_sys->app_status); // for software-generated messages (MSG_TCP), check only the payload size
	else
		is_valid_msg_test = is_valid_cmd_code_tcp_app(cmd_code, rxPayload,
			lowerlimb_sys->system_state, lowerlimb_sys->activity_state, &lowerlimb_sys->app_status);

	////////////////////////////////////////////////////////////////////////////////
	// ITM console output:
	////////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_VALID_CMD_CHECK
		if (is_valid_msg_test) {
			if (use_software_msg_list)
				printf("   <<is_valid_rcv_data_cmd_code()>> cmd code (%d) [%s] (previous cmd code (%d) [%s]) \n\n",
						cmd_code, MSG_TCP_STR[cmd_code], *cmd_code_ref, MSG_TCP_STR[*cmd_code_ref]);
			else
				printf("   <<is_valid_rcv_data_cmd_code()>> cmd code (%d) [%s] (previous cmd code (%d) [%s]) \n\n",
						cmd_code, CMD_STR[cmd_code], *cmd_code_ref, CMD_STR[*cmd_code_ref]);
		}
	#endif

	////////////////////////////////////////////////////////////////////////////////
	// Assign command code to reference variable (CRITICAL):
	////////////////////////////////////////////////////////////////////////////////

	if (is_valid_msg_test)
		*cmd_code_ref = cmd_code;
	else if (use_software_msg_list)
		*cmd_code_ref = NO_MSG_TCP;
	else
		*cmd_code_ref = NO_CMD;

	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	// Assign payload values to system info (CRITICAL):
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////
	// Exercise mode:
	////////////////////////////////////////////////////////////////////////////////

	if (use_software_msg_list) {
		#if USE_ITM_EXERC_MODE_CHECK
			printf("   <<is_valid_rcv_data_cmd_code()>> EXERCISE MODE assignments for software app (MSG_TCP) NOT IMPLEMENTED! \n\n", cmd_code);
		#endif
	}
	else {
		if (*cmd_code_ref == START_EXERCISE_CMD) {
			exercise_mode = get_exercise_mode_tcp_app(tcp_rx);

			// Reject non-applicable cases:
			if (exercise_mode == ImpedanceCtrl || exercise_mode ==	AdmittanceCtrl)
				exercise_mode = NoCtrl;
		}
	}

	lowerlimb_sys->exercise_mode = exercise_mode;

	////////////////////////////////////////////////////////////////////////////////
	// ITM console output:
	////////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_VALID_CMD_CHECK
		if (!is_valid_msg_test)
			printf("   <<is_valid_rcv_data_cmd_code()>> invalid command code received [%d] (payload problem?) \n\n", cmd_code);
	#endif

	////////////////////////////////////////////////////////////////////////////////
	// Return validity state:
	////////////////////////////////////////////////////////////////////////////////

	return is_valid_msg_test;
}

///////////////////////////////////////////////////////////////////////
// Message validation functions, low-level:
///////////////////////////////////////////////////////////////////////

uint8_t
is_valid_w5500_msg(uint8_t tcp_rx_data[]) {

	uint16_t n_cnt     = 0;
	uint8_t tmp_chksum = 0;

	const uint8_t startByte0_index = 0;
	const uint8_t startByte1_index = 1;

	uint8_t checkSum_index;
	uint8_t endByte0_index;
	uint8_t endByte1_index;

	const uint8_t msgId_index = 2;

	uint16_t cmd_code = 0;

	// memset(tcp_rx_data, 0, sizeof(tcp_rx_data));
	tcpRxLen = ethernet_w5500_rcv_data(tcp_rx_data);

	checkSum_index = tcpRxLen - 4;
	endByte0_index = tcpRxLen - 3;
	endByte1_index = tcpRxLen - 2;

	// Check for preample and postample:
	if (	tcp_rx_data[startByte0_index] != PREAMP_TCP[0]  ||
			tcp_rx_data[startByte1_index] != PREAMP_TCP[1]  ||
			tcp_rx_data[endByte0_index]   != POSTAMP_TCP[0] ||
			tcp_rx_data[endByte1_index]   != POSTAMP_TCP[1] ) {
		lowerlimb_sys_info.app_status = 1;
		return 0;
	}

	/////////////////////////////////
	// Error checksum comparison:
	/////////////////////////////////

	tmp_chksum = 0;
	for (n_cnt = 2; n_cnt < checkSum_index; n_cnt++) {
		tmp_chksum ^= tcp_rx_data[n_cnt];
	}

	if (tmp_chksum != tcp_rx_data[checkSum_index]) { // failed comparison
		send_error_msg(cmd_code, ERR_CHECKSUM_FAILED); // why do we send a message with a command code that hasn't been assigned a value?
		lowerlimb_sys_info.app_status = ERR_CHECKSUM_FAILED + 3;
		return 0;
	}

	/////////////////////////////////
	// Check if command message. Rest is discarded:
	/////////////////////////////////

	if (tcp_rx_data[msgId_index] != CMD_MSG_TYPE) {
		lowerlimb_sys_info.app_status = 2;
		return 0;
	}

	return 1;
}

#define TEST_CMD_CODE_TCP_APP_PAYLOAD 			1
#define TEST_CMD_CODE_TCP_APP_SYS_ON 			0
#define TEST_CMD_CODE_TCP_APP_CTRL_MODE 		1
#define TEST_CMD_CODE_TCP_APP_START_STOP_SYS	0
#define TEST_CMD_CODE_TCP_APP_STOP_EXE			0

uint8_t
is_valid_cmd_code_tcp_app(uint16_t cmd_code, uint8_t rxPayload, uint8_t system_state, uint8_t activity_state, uint16_t* app_status) {

	if (cmd_code == NO_CMD)
		return 0;

	#if USE_ITM_VALID_CMD_CHECK
		printf("   <<is_valid_cmd_code_tcp_app()>>:\n");
		printf("   cmd_code     = [%s]\n",   CMD_STR[cmd_code]);
		printf("   system_state = [%s]\n",   SYS_STATE_STR[system_state]);
		printf("   app_status   = [%d]\n\n", *app_status);
	#endif

	////////////////////////////////////////////////////////////////////////////////
	// CHECK PAYLOAD SIZES:
	////////////////////////////////////////////////////////////////////////////////

	#if TEST_CMD_CODE_TCP_APP_PAYLOAD
		// Payload size: 1
		if (cmd_code == START_SYS_CMD		||
			cmd_code == BRAKES_CMD			||
			cmd_code == AUTO_CALIB_MODE_CMD ||
			cmd_code == STOP_SYS_CMD		||
			cmd_code == STOP_EXERCISE_CMD		||
			cmd_code == RESET_SYS_CMD		||
			cmd_code == READ_DEV_ID_CMD		||
			cmd_code == READ_SYS_INFO_CMD 	||
			cmd_code == PAUSE_EXERCISE_CMD		||
			cmd_code == TOGGLE_SAFETY_CMD
			)
				if (!valid_app_status(rxPayload, 1,
					app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
						return 0;

		// Payload size:
		// else if () { }
	#endif

    ////////////////////////////////////////////////////////////////////////////////
	// CHECK IF SYSTEM IS ACTIVE:
    ////////////////////////////////////////////////////////////////////////////////

	#if TEST_CMD_CODE_TCP_APP_SYS_ON
		if (cmd_code == BRAKES_CMD           ||
			cmd_code == AUTO_CALIB_MODE_CMD  ||
			cmd_code == START_EXERCISE_CMD ||
			cmd_code == STOP_EXERCISE_CMD		 ||
			cmd_code == PAUSE_EXERCISE_CMD        ||
			cmd_code == TOGGLE_SAFETY_CMD    ||
			cmd_code == SET_CTRLPARAMS       ||
			cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD ||
			cmd_code == SET_TARG_PARAM_ADMCTRL_CMD   ||
			cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD
			)
				if (!valid_app_status(system_state, SYS_ON,
					app_status, cmd_code, ERR_SYSTEM_OFF, ERR_OFFSET))
						return 0;
	#endif

    ////////////////////////////////////////////////////////////////////////////////
	// VALIDATE CONTROL MODES:
	////////////////////////////////////////////////////////////////////////////////

	uint8_t exercise_mode = 0;

	#if TEST_CMD_CODE_TCP_APP_CTRL_MODE
		if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD)
			exercise_mode = PassiveTrajectoryCtrl;
		else if (cmd_code == SET_TARG_PARAM_ADMCTRL_CMD)
			exercise_mode = AdmittanceCtrl;
		else if (cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD)
			exercise_mode = ActiveTrajectoryCtrl;

		if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD ||
			cmd_code == SET_TARG_PARAM_ADMCTRL_CMD   ||
			cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD) {
				if (!valid_app_status(activity_state, IDLE,
					app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET))
						return 0;

				if (!valid_app_status(activity_state, CALIB,
					app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET))
						return 0;

				if (activity_state == EXERCISE) {
					if (!valid_app_status(exercise_mode, exercise_mode,
							app_status, cmd_code, ERR_INVALID_EXERCISE_MODE, ERR_OFFSET))
								return 0;
			}
		}
	#endif

    ////////////////////////////////////////////////////////////////////////////////
	// VALIDATE SYS COMMANDS (deactivated for LL2):
    ////////////////////////////////////////////////////////////////////////////////

	#if TEST_CMD_CODE_TCP_APP_START_STOP_SY
		if (cmd_code == START_SYS_CMD) { //start system
			// Succeeded to set system ON
			if (!valid_app_status(system_state, OFF,
				app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
					return 0;
		}

		if (cmd_code == STOP_SYS_CMD) { //stop system
			if (!valid_app_status(system_state, ON,
				app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
					return 0;
		}
	#endif

    ////////////////////////////////////////////////////////////////////////////////
	// VALIDATE STOP COMMANDS (deactivated for LL2):
    ////////////////////////////////////////////////////////////////////////////////

	#if TEST_CMD_CODE_TCP_APP_STOP_EXE
		if (cmd_code == STOP_EXERCISE_CMD ||	cmd_code == SET_CTRLPARAMS)
			if (!valid_app_status(activity_state, CALIB,
					app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET) ||
				!valid_app_status(activity_state, IDLE,
					app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET) ||
				!valid_app_status(exercise_state, STOPPED,
					app_status, cmd_code, ERR_EXERCISE_NOT_RUNNING, ERR_OFFSET) )
						return 0;
	#endif

    ////////////////////////////////////////////////////////////////////////////////
	// COMMAND CODE IS VALID:
    ////////////////////////////////////////////////////////////////////////////////

	return 1;
}

uint8_t
is_valid_payload_size_software(uint16_t cmd_code, uint8_t rxPayload, uint16_t* app_status) {

	uint8_t n_payload;

	if (cmd_code == NO_MSG_TCP)
		return 0;

	#if USE_ITM_VALID_CMD_CHECK
		printf("   <<is_valid_payload_size_software()>>:\n");
		printf("   cmd_code   = [%s] (%d)\n", MSG_TCP_STR[cmd_code], cmd_code);
		printf("   app_status = [%d]\n\n",    *app_status);
	#endif

	// Validate command code:
	if (cmd_code == Connect_To_Robot_MSG_TCP	||
		cmd_code == Calibrate_Robot_MSG_TCP		||
		cmd_code == Move_To_Start_MSG_TCP		||
		cmd_code == Robot_Shutdown_MSG_TCP		||
		cmd_code == Start_Exercise_MSG_TCP		||
		cmd_code == Stop_Exercise_MSG_TCP		||
		cmd_code == Stdby_Start_Point_MSG_TCP)
			n_payload = N_PAYL_Def_MSG_TCP;

	else if (cmd_code == Go_To_Exercise_MSG_TCP)
			n_payload = N_PAYL_Go_To_Exercise_MSG_TCP;

	else if (cmd_code == Pedal_Travel_MSG_TCP)
			n_payload = N_PAYL_Pedal_Travel_MSG_TCP;

	else if (cmd_code == F_Therapy_Change_MSG_TCP)
			n_payload = N_PAYL_F_Therapy_Change_MSG_TCP;

	// Validate payload size:
	if (!valid_app_status(rxPayload, n_payload,
		app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET))
			return 0;
	else
			return 1;
}

/**
 @brief: Send calibration response message
 @param[in]: actProto = current active calibration protocol
 @param[in]: percent = 0 - 100%. Setting to 100 will stop calibration mode.
 @param[in]: state = 0 (0 = idle, 1 = running, 2 = passed, 3 = failed)
 */

uint8_t
send_calibration_resp(uint8_t actProto, uint8_t percent, uint8_t state) {
	uint8_t tmp_resp_msg[2];

	//update calibration state
	if (state == 1)
		lowerlimb_sys_info.calibrate_state = 1;
	else
		lowerlimb_sys_info.calibrate_state = 0;

	//check whether to update isCalibrated
	if (state == 2)
		lowerlimb_sys_info.isCalibrated = YES;

	//reset activity to IDLE:
	if (percent == 100)
		set_activity_idle();

	tmp_resp_msg[0] = percent;
	tmp_resp_msg[1] = state;

	return send_resp_msg(AUTO_CALIB_MODE_CMD, tmp_resp_msg, 2);
}

/**
 @brief: Function to just send OK response
 @param[in]: tmp_cmd_code  = command code.
 @retval: 0 = success, 1 = failed
 */

uint8_t
send_OK_resp(uint16_t tmp_cmd_code) {
	uint8_t tmpTxData[1] = { 0 };

	return send_resp_msg(tmp_cmd_code, tmpTxData, 1);
}

/**
 @brief: tx error message. All message is MSB first.
 @retval: 0 = success, 1 = failed
 */

uint8_t
send_error_msg(uint16_t tmp_cmd_code, uint16_t tmp_err_code) {
	uint8_t tcp_err_msg[13];
	uint8_t err_chksum_index;

	memset(tcp_err_msg, 0, sizeof(tcp_err_msg));    //clear

	//set preample and postample
	tcp_err_msg[0] = PREAMP_TCP[0];
	tcp_err_msg[1] = PREAMP_TCP[1];
	tcp_err_msg[10] = POSTAMP_TCP[0];
	tcp_err_msg[11] = POSTAMP_TCP[1];
	tcp_err_msg[12] = 0; //encrypted byte

	//set payload
	tcp_err_msg[2] = ERROR_MSG_TYPE;
	tcp_err_msg[3] = (uint8_t) (tmp_cmd_code >> 8);
	tcp_err_msg[4] = (uint8_t) tmp_cmd_code;
	tcp_err_msg[5] = 0;
	tcp_err_msg[6] = 2;
	tcp_err_msg[7] = (uint8_t) (tmp_err_code >> 8);
	tcp_err_msg[8] = (uint8_t) tmp_err_code;

	//set checksum
	for (err_chksum_index = 2; err_chksum_index <= 8; err_chksum_index++)
		tcp_err_msg[9] ^= tcp_err_msg[err_chksum_index];

	//send TCP
	ethernet_w5500_send_data(tcp_err_msg, sizeof(tcp_err_msg));

	return 0;
}

/**
 @brief: tx response message. All message tmp_payload is expected to be MSB first.
 @param[in]: tmp_cmd_code  = command code.
 @param[in]: tmp_payload  = pointer to transmission data buffer
 @param[in]: tmp_len  = len of data buffer
 @retval: 0 = success, 1 = failed
 */

uint8_t
send_resp_msg(uint16_t tmp_cmd_code, uint8_t tmp_payload[],
		uint16_t tmp_len) {
	uint8_t tcp_resp_msg[tmp_len + 11];
	uint16_t err_chksum_index;

	memset(tcp_resp_msg, 0, sizeof(tcp_resp_msg)); //clear

	//set preample and postample
	tcp_resp_msg[0] = PREAMP_TCP[0];
	tcp_resp_msg[1] = PREAMP_TCP[1];
	tcp_resp_msg[8 + tmp_len] = POSTAMP_TCP[0];
	tcp_resp_msg[9 + tmp_len] = POSTAMP_TCP[1];
	tcp_resp_msg[10 + tmp_len] = 0; //encrypted byte

	//set payload
	tcp_resp_msg[2] = RESP_MSG_TYPE;
	tcp_resp_msg[3] = (uint8_t) (tmp_cmd_code >> 8);
	tcp_resp_msg[4] = (uint8_t) tmp_cmd_code;
	tcp_resp_msg[5] = (uint8_t) (tmp_len >> 8);
	tcp_resp_msg[6] = (uint8_t) tmp_len;

	//tmp_payload is expected to be MSB first, hence,
	//this memcpy will just copy the data according to address w/o any further rearrangement.
	memcpy(&tcp_resp_msg[7], tmp_payload, tmp_len);

	//set checksum
	for (err_chksum_index = 2; err_chksum_index < (7 + tmp_len); err_chksum_index++){
		tcp_resp_msg[7 + tmp_len] ^= tcp_resp_msg[err_chksum_index];
	}

	//send TCP
	ethernet_w5500_send_data(tcp_resp_msg, sizeof(tcp_resp_msg));

	return 0;
}

uint8_t
send_lowerlimb_exercise_feedback_help(uint64_t up_time,
								float f_x, float f_y,
								int32_t fQei_L, int32_t fQei_R,
								float fVel_X, float fVel_Y,
								float fVolt_L, float fVolt_R,
								float fCs_L, float fCs_R,
								float fFs_X, float fFs_Y,
								float fCmd_Fx, float fCmd_Fy, float fCmd_Fr,
								float fRefPos_x, float fRefPos_y,
								float fRefVel_x, float fRefVel_y,
								float fRefPhase, float fRefFreq) {

	const int LEN_DATA = 91;

	uint8_t resp_payload_index = 0;
	uint8_t tmp_resp_msg[LEN_DATA];
	uint32_t timestamp = (uint32_t) up_time;
	// float empty_float = 0.0f;

	///////////////////////////////////////////////////////////////////////////
	// Time stamp:
	///////////////////////////////////////////////////////////////////////////

	memcpy_msb(&tmp_resp_msg[resp_payload_index], &timestamp, sizeof(timestamp));
	resp_payload_index += sizeof(timestamp);

	///////////////////////////////////////////////////////////////////////////
	// End effector measured position:
	///////////////////////////////////////////////////////////////////////////

	//X-axis Position
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &f_x, sizeof(f_x));
	resp_payload_index += sizeof(f_x);
	//Y-axis Position
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &f_y, sizeof(f_y));
	resp_payload_index += sizeof(f_y);

	///////////////////////////////////////////////////////////////////////////
	// Encoder positions:
	///////////////////////////////////////////////////////////////////////////

	//Quadrature Encoder Interface (Left)
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fQei_L, sizeof(fQei_L));
	resp_payload_index += sizeof(fQei_L);
	//Quadrature Encoder Interface (Right)
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fQei_R, sizeof(fQei_R));
	resp_payload_index += sizeof(fQei_R);

	///////////////////////////////////////////////////////////////////////////
	// End effector measured	velocity:
	///////////////////////////////////////////////////////////////////////////

	//X-axis Velocity
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fVel_X, sizeof(fVel_X));
	resp_payload_index += sizeof(fVel_X);
	//Y-axis Velocity
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fVel_Y, sizeof(fVel_Y));
	resp_payload_index += sizeof(fVel_Y);

	///////////////////////////////////////////////////////////////////////////
	// Voltage commands:
	///////////////////////////////////////////////////////////////////////////

	//Voltage Left
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fVolt_L, sizeof(fVolt_L));
	resp_payload_index += sizeof(fVolt_L);
	//Voltage Right
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fVolt_R, sizeof(fVolt_R));
	resp_payload_index += sizeof(fVolt_R);

	///////////////////////////////////////////////////////////////////////////
	// Motor current:
	///////////////////////////////////////////////////////////////////////////

	//Current sense (Left)
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fCs_L, sizeof(fCs_L));
	resp_payload_index += sizeof(fCs_L);
	//Current sense (Right)
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fCs_R, sizeof(fCs_R));
	resp_payload_index += sizeof(fCs_R);

	///////////////////////////////////////////////////////////////////////////
	// End-effector measured force:
	///////////////////////////////////////////////////////////////////////////

	//X Force
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fFs_X, sizeof(fFs_X));
	resp_payload_index += sizeof(fFs_X);
	//Y Force
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fFs_Y, sizeof(fFs_Y));
	resp_payload_index += sizeof(fFs_Y);

	///////////////////////////////////////////////////////////////////////////
	// End-effector commanded force:
	///////////////////////////////////////////////////////////////////////////

	//Commanded Force on X-axis
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fCmd_Fx, sizeof(fCmd_Fx));
	resp_payload_index += sizeof(fCmd_Fx);
	//Commanded Force on Y-axis
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fCmd_Fy, sizeof(fCmd_Fy));
	resp_payload_index += sizeof(fCmd_Fy);
	//Commanded Force
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fCmd_Fr, sizeof(fCmd_Fr));
	resp_payload_index += sizeof(fCmd_Fr);

	///////////////////////////////////////////////////////////////////////////
	// Operation state:
	///////////////////////////////////////////////////////////////////////////

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.operation_state;
	resp_payload_index += sizeof(lowerlimb_sys_info.operation_state);

	///////////////////////////////////////////////////////////////////////////
	// Empty state (was Force Sensor?):
	///////////////////////////////////////////////////////////////////////////

	tmp_resp_msg[resp_payload_index] = 0;
	resp_payload_index += sizeof(lowerlimb_sys_info.operation_state);

	///////////////////////////////////////////////////////////////////////////
	// Brakes state:
	///////////////////////////////////////////////////////////////////////////

	//Left Brake and Right Brake
	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.l_brake_status
			+ (lowerlimb_sys_info.r_brake_status << 4);
	resp_payload_index += sizeof(lowerlimb_sys_info.l_brake_status);

	///////////////////////////////////////////////////////////////////////////
	// End-effector reference position:
	///////////////////////////////////////////////////////////////////////////

	//End Effector Reference Position X
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fRefPos_x, sizeof(fRefPos_x));
	resp_payload_index += sizeof(fRefPos_x);
	//End Effector Reference Position Y
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fRefPos_y, sizeof(fRefPos_y));
	resp_payload_index += sizeof(fRefPos_y);

	///////////////////////////////////////////////////////////////////////////
	// End-effector reference velocity:
	///////////////////////////////////////////////////////////////////////////

	//End Effector Reference Velocity X
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fRefVel_x, sizeof(fRefVel_x));
	resp_payload_index += sizeof(fRefVel_x);
	//End Effector Reference Velocity Y
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fRefVel_y, sizeof(fRefVel_y));
	resp_payload_index += sizeof(fRefVel_y);

	///////////////////////////////////////////////////////////////////////////
	// End-effector reference phase and frequency:
	///////////////////////////////////////////////////////////////////////////

	//Cyclic Trajectory Phase
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fRefPhase, sizeof(fRefPhase));
	resp_payload_index += sizeof(fRefPhase);
	//Cyclic Trajectory Instantaneous Frequency
	memcpy_msb(&tmp_resp_msg[resp_payload_index], &fRefFreq, sizeof(fRefFreq));

	send_resp_msg(0x09, tmp_resp_msg, LEN_DATA);

	return 0;
}

///////////////////////////////////////////////////////////////////////
// Helper functions:
///////////////////////////////////////////////////////////////////////

uint8_t
send_lowerlimb_exercise_feedback(uint64_t up_time, lowerlimb_mech_readings_t* mech_readings, lowerlimb_motors_settings_t* motor_settings,
		lowerlimb_ref_kinematics_t* ref_kinematics) {

	float FORCE_END_MAGN = 0.0; // don't need to maintain this

	return send_lowerlimb_exercise_feedback_help(
		up_time,
		mech_readings->coord.x,
		mech_readings->coord.y,
		qei_count_L_read(), // mech_readings->left.qei_count,
		qei_count_R_read(), // mech_readings->right.qei_count,
		mech_readings->velocity.x,
		mech_readings->velocity.y,
		motor_settings->left.volt,
		motor_settings->right.volt,
		mech_readings->left.currsens_amps,
		mech_readings->right.currsens_amps,
		mech_readings->Xforce, // X-axis Force Sensor
		mech_readings->Yforce, // Y-axis Force Sensor
		motor_settings->force_end[IDX_X],
		motor_settings->force_end[IDX_Y],
		FORCE_END_MAGN,
		ref_kinematics->p_ref[IDX_X],
		ref_kinematics->p_ref[IDX_Y],
		ref_kinematics->dt_p_ref[IDX_X],
		ref_kinematics->dt_p_ref[IDX_Y],
		ref_kinematics->phi_ref,
		ref_kinematics->dt_phi_ref);
}

void
send_lowerlimb_sys_info(uint8_t tmp_resp_msg[], uint16_t cmd_code) { //was send_lowerlimb_sys_info(lowerlimb_sys_info_t* lowerlimb_sys_info,

	uint16_t resp_payload_index = 0;
	uint16_t LEN_BYTES_RESERVE = 40;

	memcpy_msb(tmp_resp_msg, &(lowerlimb_sys_info.unix), sizeof(lowerlimb_sys_info.unix));
	resp_payload_index += sizeof(lowerlimb_sys_info.unix);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.system_state;
	resp_payload_index += sizeof(lowerlimb_sys_info.system_state);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.exercise_state;
	resp_payload_index += sizeof(lowerlimb_sys_info.exercise_state);

	if (lowerlimb_sys_info.calibrate_state == 1)
		tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.calibrate_state;
	else if (lowerlimb_sys_info.isCalibrated == YES)
		tmp_resp_msg[resp_payload_index] = 0;
	else
		tmp_resp_msg[resp_payload_index] = 0xFF; //indicate need calibration

	resp_payload_index += sizeof(lowerlimb_sys_info.calibrate_state);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.operation_state;
	resp_payload_index += sizeof(lowerlimb_sys_info.operation_state);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.fw_maj_ver;
	resp_payload_index += sizeof(lowerlimb_sys_info.fw_maj_ver);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.fw_min_ver;
	resp_payload_index += sizeof(lowerlimb_sys_info.fw_min_ver);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.fw_patch_ver;
	resp_payload_index += sizeof(lowerlimb_sys_info.fw_patch_ver);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.safetyOFF;
	resp_payload_index += sizeof(lowerlimb_sys_info.safetyOFF);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.l_brake_status;
	resp_payload_index += sizeof(lowerlimb_sys_info.l_brake_status);

	tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.r_brake_status;
	resp_payload_index += sizeof(lowerlimb_sys_info.r_brake_status);

	memcpy_msb(&tmp_resp_msg[resp_payload_index], &lowerlimb_sys_info.f_x, sizeof(lowerlimb_sys_info.f_x));
	resp_payload_index += sizeof(lowerlimb_sys_info.f_x);

	memcpy_msb(&tmp_resp_msg[resp_payload_index], &lowerlimb_sys_info.f_y, sizeof(lowerlimb_sys_info.f_y));
	resp_payload_index += sizeof(lowerlimb_sys_info.f_y);

	resp_payload_index +=  LEN_BYTES_RESERVE; //Reserved bytes

	//send response:
	send_resp_msg(cmd_code, tmp_resp_msg, resp_payload_index);
}

/**
 @brief: reset the lowerlimb_sys_info
 @retval: none
 */
void reset_lowerlimb_sys_info(void) {
	//store old values
	uint8_t fw_maj = lowerlimb_sys_info.fw_maj_ver;
	uint8_t fw_min = lowerlimb_sys_info.fw_min_ver;
	memset(&lowerlimb_sys_info, 0, sizeof(lowerlimb_sys_info));

	//forced system ID
	memcpy(lowerlimb_sys_info.device_id, DEVICE_SERIAL_ID, 25);

	//reset unix to 01-01-1970 00:00:00
	lowerlimb_sys_info.unix = 0;

	//restore old values
	lowerlimb_sys_info.fw_maj_ver = fw_maj;
	lowerlimb_sys_info.fw_min_ver = fw_min;
}

uint8_t valid_app_status(uint8_t property, uint8_t value, uint16_t* app_status, uint16_t cmd_code, uint16_t ERR_CODE, uint16_t err_offset) {
	if (property != value) {
		send_error_msg(cmd_code, ERR_CODE);
		*app_status = ERR_CODE + err_offset;

		#if USE_ITM_VALID_CMD_CHECK
			if (ERR_CODE > LEN_ERR_LIST - 1)
				ERR_CODE = LEN_ERR_LIST - 1;

			printf("\n");
			printf("valid_app_status() ERROR: cmd_code [%d] generated error [%s]\n\n", cmd_code, ERR_STR[ERR_CODE]);
		#endif

		return 0;
	}
	else
		return 1;
}

/**
 @brief: set Unix time
 @retval: 0 = success, 1 = failed
 */
uint8_t set_unix_time(uint64_t tmp_unix) {
	//check unix valid (must be lesser than 01-01-3040 00:00:00
	if (tmp_unix >= 33765897600)
		return 1;

	lowerlimb_sys_info.unix = tmp_unix;

	//set RTC
	set_TIM_unix(lowerlimb_sys_info.unix);

	return 0;
}

/**
 @brief: update Unix time with RTC value
 @retval: 0 = success
 */
uint8_t update_unix_time(void) {
	if (lowerlimb_sys_info.unix != get_TIM_unix())
		lowerlimb_sys_info.unix = get_TIM_unix();

	return 0;
}

void update_operation_state(uint8_t* operation_state, uint8_t ui8EBtnState, uint8_t ui8Alert) {
	if (*operation_state == 0)
		*operation_state |= OPS_STATUS_NORMAL_MASK;

	if (ui8Alert == 1) {
		*operation_state |= OPS_STATUS_SAMPLING_ALERT_MASK;
		*operation_state &= ~OPS_STATUS_NORMAL_MASK;
	} else if (ui8Alert == 2) {
		*operation_state |= OPS_STATUS_RT_VEL_ALERT_MASK;
		*operation_state &= ~OPS_STATUS_NORMAL_MASK;
	}

	if (ui8EBtnState == 0)
		*operation_state |= OPS_STATUS_EBTN_PRESSED_MASK;
	else
		*operation_state &= ~OPS_STATUS_EBTN_PRESSED_MASK;
}

/**
 @brief: set sytem to ON
 @retval: 0 = success, 1 = failed
 */
uint8_t set_system_on(void) {
	lowerlimb_sys_info.system_state = SYS_ON;
	return 0;
}

/**
 @brief: set sytem to OFF
 @retval: 0 = success, 1 = failed
 */
uint8_t set_system_off(void) {
	lowerlimb_sys_info.system_state = SYS_OFF;

	//reset sys info
	reset_lowerlimb_sys_info();

	return 0;
}

/**
 @brief: set activity to IDLE
 @retval: 0 = success, 1 = failed
 */
uint8_t set_activity_idle(void) {
	lowerlimb_sys_info.activity_state = IDLE;
	return 0;
}

/**
 @brief: set activity to EXERCISE
 @retval: 0 = success, 1 = failed
 */
uint8_t set_activity_exercise(void) {
	if (lowerlimb_sys_info.activity_state != IDLE)
		return 1;

	lowerlimb_sys_info.activity_state = EXERCISE;
	return 0;
}

/**
 @brief: set activity to CALIB
 @retval: 0 = success, 1 = failed
 */
uint8_t set_activity_calib(void) {
	if (lowerlimb_sys_info.activity_state != IDLE)
		return 1;

	lowerlimb_sys_info.activity_state = CALIB;
	return 0;
}

/**
 @brief: stop exercise
 @retval: 0 = success, 1 = failed
 */
uint8_t stop_exercise(lowerlimb_motors_settings_t* LL_motors_settings) {
	//set activity to IDLE
	if (set_activity_idle() != 0) {
		return 1;
	}

	//set exercise_state to STOPPED
	lowerlimb_sys_info.exercise_state = STOPPED;

	//reset
	clear_lowerlimb_motors_settings(LL_motors_settings);

	return 0;
}

void reset_emergency_alerts(void) {
	lowerlimb_sys_info.operation_state &= ~(OPS_STATUS_SAMPLING_ALERT_MASK
			| OPS_STATUS_RT_VEL_ALERT_MASK); //Reset Emergency Alerts
	lowerlimb_sys_info.operation_state |= OPS_STATUS_NORMAL_MASK;
}

bool get_l_brake_cmd(void) {
	return lowerlimb_brakes_command.l_brake_disengage;
}

bool get_r_brake_cmd(void) {
	return lowerlimb_brakes_command.r_brake_disengage;
}

void set_l_brake_status(uint8_t status) {
	lowerlimb_sys_info.l_brake_status = status;
}

void set_r_brake_status(uint8_t status) {
	lowerlimb_sys_info.r_brake_status = status;
}

void
set_brakes_simple() {
	set_l_brake_status(l_brakes(get_l_brake_cmd()));
	set_r_brake_status(r_brakes(get_r_brake_cmd()));
}

void
set_brakes_timed(uint64_t uptime, uint64_t* brakes_next_time) {
	if (uptime >= *brakes_next_time) {
		*brakes_next_time = uptime + 1; // 1kHz

		set_l_brake_status(l_brakes(get_l_brake_cmd() && Read_Haptic_Button()));
		set_r_brake_status(r_brakes(get_r_brake_cmd() && Read_Haptic_Button()));
	}
}

exercise_mode_t
get_exercise_mode_tcp_app(uint8_t tcp_rx[]) {
	return (exercise_mode_t)tcp_rx[payloadStart_index];
}

/**
 @brief: memcpy function for MSB. This is a helper function.
 @retval: dest pointer address
 */
void* memcpy_msb(void *pDest, const void *pSrc, unsigned long len) {
	char *d = pDest;
	d += (len - 1);
	const char *s = pSrc;
	while (len--)
		*d-- = *s++;

	return pDest;
}
/* [] END OF FILE */
