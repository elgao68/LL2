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
#include <lowerlimb_tcp_app.h>
#include <stdlib.h>
#include <string.h>
#include "w5500_app.h"
#include "timer.h"

#if (MOTOR_DRIVER_OPS)
	#include "qei_motor_drivers.h"
#endif

//Serial Number
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
//message preample and postample
const uint8_t PREAMP_TCP[2] = { 0x48, 0x4D };
const uint8_t POSTAMP_TCP[2] = { 0x68, 0x6D };

//**********************************************
//Private variables
//**********************************************
static lowerlimb_sys_info_t lowerlimb_sys_info;
//static uint64_t ui64CalibNextTime = 0;
static lowerlimb_brakes_command_t lowerlimb_brakes_command;
//TCP messages
static uint8_t tcpRxData[DATA_BUF_SIZE];
static uint16_t tcpRxLen = 0;
extern ADC_HandleTypeDef hadc3;

//**********************************************
//Private prototypes
//**********************************************
void reset_lowerlimb_sys_info(void);
// void clear_lowerlimb_targs_params(void);
uint8_t update_unix_time(void);
uint8_t set_unix_time(uint64_t tmp_unix);
uint8_t set_system_on(void);
uint8_t set_system_off(void);
uint8_t set_activity_idle(void);
uint8_t set_activity_exercise(void);
uint8_t set_activity_calib(void);
uint8_t send_resp_msg(uint16_t tmp_cmd_code, uint8_t tmp_payload[],
		uint16_t tmp_len);
uint8_t send_error_msg(uint16_t tmp_cmd_code, uint16_t tmp_err_code);
uint8_t send_OK_resp(uint16_t tmp_cmd_code);

//**********************************************
//Functions
//**********************************************
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

uint8_t set_lowerlimb_exercise_feedback_info(uint64_t up_time,
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

/**
 @brief: Send calibration response message
 @param[in]: actProto = current active calibration protocol
 @param[in]: percent = 0 - 100%. Setting to 100 will stop calibration mode.
 @param[in]: state = 0 (0 = idle, 1 = running, 2 = passed, 3 = failed)
 */
uint8_t send_calibration_resp(uint8_t actProto, uint8_t percent, uint8_t state) {
	uint8_t tmp_resp_msg[2];

	//update calibration state
	if (state == 1)
		lowerlimb_sys_info.calibrate_state = 1;
	else
		lowerlimb_sys_info.calibrate_state = 0;

	//check whether to update isCalibrated
	if (state == 2)
		lowerlimb_sys_info.isCalibrated = YES;

	if (percent == 100) {
		//reset acitivity to IDLE
		set_activity_idle();
	}
	tmp_resp_msg[0] = percent;
	tmp_resp_msg[1] = state;

	return send_resp_msg(AUTO_CALIB_MODE_CMD, tmp_resp_msg, 2);
}

/**
 @brief: set sytem to ON
 @retval: 0 = success, 1 = failed
 */
uint8_t set_system_on(void) {
	lowerlimb_sys_info.system_state = ON;
	return 0;
}

/**
 @brief: set sytem to OFF
 @retval: 0 = success, 1 = failed
 */
uint8_t set_system_off(void) {
	lowerlimb_sys_info.system_state = OFF;

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

/**
 @brief: tx response message. All message tmp_payload is expected to be MSB first.
 @param[in]: tmp_cmd_code  = command code.
 @param[in]: tmp_payload  = pointer to transmission data buffer
 @param[in]: tmp_len  = len of data buffer
 @retval: 0 = success, 1 = failed
 */
uint8_t send_resp_msg(uint16_t tmp_cmd_code, uint8_t tmp_payload[],
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

/**
 @brief: Function to just send OK response
 @param[in]: tmp_cmd_code  = command code.
 @retval: 0 = success, 1 = failed
 */
uint8_t send_OK_resp(uint16_t tmp_cmd_code) {
	uint8_t tmpTxData[1] = { 0 };

	return send_resp_msg(tmp_cmd_code, tmpTxData, 1);
}

/**
 @brief: tx error message. All message is MSB first.
 @retval: 0 = success, 1 = failed
 */
uint8_t send_error_msg(uint16_t tmp_cmd_code, uint16_t tmp_err_code) {
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
 @brief: Start in the APP.
 @retval: 0 = success, 1 = failed
 */
uint8_t lowerlimb_tcp_init_app_state(uint64_t init_unix, uint8_t maj_ver,
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

/**
 @brief: Run in the main().
 @param[in]: ui8EBtnState = emergency button (1 = normal, 0 = asserted)
 @param[in]: ui8Alert = alert msg from motor algo (0 = no alert, 1 = sampling alert, 2 = RT vel alert)
 @retval: lowerlimb_sys_info_t
 */

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// TCP/IP APP STATE - GAO
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

lowerlimb_sys_info_t lowerlimb_tcp_app_state(uint8_t ui8EBtnState, uint8_t ui8Alert,
		traj_ctrl_params_t* traj_ctrl_params, admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings) {

	uint16_t n_cnt = 0;
	uint8_t tmp_chksum = 0;
	//uint8_t tmp_index = 0;
	uint8_t tmp_chk = 0;
	uint16_t cmd_code = 0;
	uint64_t ui64_tmp = 0;
	uint8_t rxPayload = 0;
	uint8_t tmp_resp_msg[465];
	//int16_t i6TmpQEIL, i6TmpQEIR;

	// Obsolete variables:
	// lowerlimb_tcp_exercise_targ_params_t tmp_parsed_targ_params;
	// lowerlimb_offset_command_t tmp_parsed_offset_params;

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
	uint16_t resp_payload_index = 0;
	//const uint8_t exercise_payload_length = 89;

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

	//Force Sensor
	uint32_t force_end_in_x_sensor = 0;
	uint32_t force_end_in_y_sensor = 0;
	uint32_t dum_force_end_in_x = 0;
	uint32_t dum_force_end_in_y = 0;
	float force_end_in_x_sensor_f = 0;
	float force_end_in_y_sensor_f = 0;

	//reset
	lowerlimb_sys_info.tcp_status = 0;
	lowerlimb_sys_info.calib_prot_req = 0;

	//**********************************
	//update system operation status
	//**********************************
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

	//**********************************
	//Update local Unix time
	//**********************************
	update_unix_time();

	//**********************************
	// Check if TCP connected
	//**********************************
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

	//**********************************
	// Parsing and executing TCP messages
	//**********************************

	//check if there's new message
	if (ethernet_w5500_new_rcv_data()) {
		//*******************************
		//get tcp data
		//*******************************
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
			lowerlimb_sys_info.tcp_status = 1;
			return lowerlimb_sys_info;
		}
		//*******************************
		//error checksum comparison
		//*******************************
		tmp_chksum = 0;
		for (n_cnt = 2; n_cnt < checkSum_index; n_cnt++) {
			tmp_chksum ^= tcpRxData[n_cnt];
		}

		if (tmp_chksum != tcpRxData[checkSum_index])  //failed comparison
				{
			send_error_msg(cmd_code, ERR_CHECKSUM_FAILED);
			lowerlimb_sys_info.tcp_status = ERR_CHECKSUM_FAILED + 3;
			return lowerlimb_sys_info;
		}

		//*******************************
		//check if command message. Rest is discarded
		//*******************************
		if (tcpRxData[msgId_index] != CMD_MSG_TYPE) {
			lowerlimb_sys_info.tcp_status = 2;
			return lowerlimb_sys_info;
		}

		//*******************************
		//parse command code
		//*******************************
		cmd_code = ((uint16_t) tcpRxData[cmdCode_index] << 8)
				+ (uint16_t) tcpRxData[cmdCode_index + 1];
		rxPayload = ((uint16_t) tcpRxData[payloadLen_index] << 8)
				+ (uint16_t) tcpRxData[payloadLen_index + 1];
		//rxPayload = tcpRxData[5];

		switch (cmd_code) {
			case SET_UNIX_CMD: //set unix
				//check if rxPayload is long enough
				if (rxPayload != sizeof(lowerlimb_sys_info.unix)) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//pack UNIX
				ui64_tmp = 0;
				memcpy_msb(&ui64_tmp, &tcpRxData[payloadStart_index], sizeof(lowerlimb_sys_info.unix));

				//check if UNIX is valid
				if (set_unix_time(ui64_tmp) != 0) {
					send_error_msg(cmd_code, ERR_INVALID_UNIX);
					lowerlimb_sys_info.tcp_status = ERR_INVALID_UNIX + 3;
					return lowerlimb_sys_info;
				}

				send_OK_resp(cmd_code);
				break;

			case START_SYS_CMD: //start system
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}
					//Succeeded to set system ON

				if (lowerlimb_sys_info.system_state != OFF){
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				reset_lowerlimb_sys_info();

				lowerlimb_sys_info.system_state = ON;
				lowerlimb_sys_info.activity_state = IDLE;
				//set exercise_state to STOPPED
				lowerlimb_sys_info.exercise_state = STOPPED;

				//reset
				// clear_lowerlimb_targs_params();

				send_OK_resp(cmd_code);
				break;

			case STOP_SYS_CMD: //stop system
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				if (lowerlimb_sys_info.system_state != ON){
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				lowerlimb_sys_info.system_state = OFF;
				lowerlimb_sys_info.activity_state = IDLE;
				//set exercise_state to STOPPED
				lowerlimb_sys_info.exercise_state = STOPPED;

				//Reset brake command
				lowerlimb_brakes_command.l_brake_disengage = false;
				lowerlimb_brakes_command.r_brake_disengage = false;

				send_OK_resp(cmd_code);
				break;

			case RESET_SYS_CMD: //restart system
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//send resp
				send_OK_resp(cmd_code);
				HAL_Delay(100); //wait for msg to be transmitted
				HAL_NVIC_SystemReset(); //reset MCU
				break;

			case READ_DEV_ID_CMD: //read device ID
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//send resp message with device ID
				send_resp_msg(cmd_code, lowerlimb_sys_info.device_id,
						sizeof(lowerlimb_sys_info.device_id));
				break;

			case READ_SYS_INFO_CMD: //read system info
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//pack data
				memcpy_msb(tmp_resp_msg, &lowerlimb_sys_info.unix, sizeof(lowerlimb_sys_info.unix));
				resp_payload_index += sizeof(lowerlimb_sys_info.unix);

				tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.system_state;
				resp_payload_index += sizeof(lowerlimb_sys_info.system_state);

				tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.exercise_state;
				resp_payload_index += sizeof(lowerlimb_sys_info.exercise_state);

				if (lowerlimb_sys_info.calibrate_state == 1) {
					tmp_resp_msg[resp_payload_index] = lowerlimb_sys_info.calibrate_state;
				} else {
					if (lowerlimb_sys_info.isCalibrated == YES)
						tmp_resp_msg[resp_payload_index] = 0;
					else
						tmp_resp_msg[resp_payload_index] = 0xFF; //indicate need calibration
				}
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

				resp_payload_index += 40; //Reserved bytes

				//send resp
				send_resp_msg(cmd_code, tmp_resp_msg, resp_payload_index);
				break;

			case AUTO_CALIB_MODE_CMD: //enter calibration
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (lowerlimb_sys_info.activity_state){
					case EXERCISE:
						send_error_msg(cmd_code, ERR_EXERCISE_ACTIVE);
						lowerlimb_sys_info.tcp_status = ERR_EXERCISE_ACTIVE + 3;
						return lowerlimb_sys_info;
						break;
					case CALIB:
						//check if the request is to terminate ongoing calibration
						if (tcpRxData[rx_payload_index] != 255) {
							send_error_msg(cmd_code, ERR_CALIB_ACTIVE);
							lowerlimb_sys_info.tcp_status = ERR_CALIB_ACTIVE + 3;
							return lowerlimb_sys_info;
						}
						break;
					case IDLE:
						//enter calibration routines.
						lowerlimb_sys_info.activity_state = CALIB;
						break;
					default:
						send_error_msg(cmd_code, ERR_GENERAL_NOK);
						lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
						return lowerlimb_sys_info;
				}

				lowerlimb_sys_info.calib_prot_req = tcpRxData[rx_payload_index];

				switch ((Calib_protocol)lowerlimb_sys_info.calib_prot_req) {

					case CalibEncoders:	//Calibrate Left Motor
						qei_count_R_reset();
						qei_count_P_reset();

						//send resp
						send_calibration_resp(lowerlimb_sys_info.calib_prot_req, 100, 2);

						//reset acitivity to IDLE
						set_activity_idle();
						break;

					case CalibForceSensor:	//Calibrate Right Motor
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
						break;

					case CalibEncodersFS:	//Calibrate Both Motors
						//Reset Encoder
						qei_count_R_reset();
						qei_count_P_reset();

						//Zero calib Force Sensor
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
						break;

					case AutoCalibEncodersFS:	//Auto Calibrate Both Axis
						//To be implemented later
						send_error_msg(cmd_code, ERR_GENERAL_NOK);
						break;

					case StopCalib: //Terminate Calibration
						//send response
						send_calibration_resp(lowerlimb_sys_info.calib_prot_req, 0, 0);

						//go back to idle
						lowerlimb_sys_info.activity_state = IDLE;
						// set_activity_idle();
						break;

					default:
						send_error_msg(cmd_code, ERR_GENERAL_NOK);
				}
				break;

			case START_RESUME_EXE_CMD: //start/resume exercise
				//check if rxPayload is long enough
				if (rxPayload != 11) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//reset emergency alerts if any
				reset_emergency_alerts();

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				/* 	If exercise_state in paused state, resume exercise.
				If activity is in IDLE state, start exercise to SETUP state.
				Rest are errors.
				*/
				switch (lowerlimb_sys_info.activity_state) {
					case IDLE:
						//check if device has been calibrated.
						if (lowerlimb_sys_info.isCalibrated == NO) {
							send_error_msg(cmd_code, ERR_CALIBRATION_NEEDED);
							lowerlimb_sys_info.tcp_status = ERR_CALIBRATION_NEEDED + 3;
							return lowerlimb_sys_info;
						}

						//Check Exercise mode
						lowerlimb_sys_info.exercise_mode = (exercise_mode_t)tcpRxData[rx_payload_index];
						rx_payload_index += 1;

						switch(lowerlimb_sys_info.exercise_mode) {
							case ImpedanceCtrl:
							case PassiveTrajectoryCtrl:
							case AdmittanceCtrl:
							case ActiveTrajectoryCtrl:
								break;
							default:
								send_error_msg(cmd_code, ERR_INVALID_EXERCISE_MODE);
								lowerlimb_sys_info.tcp_status = ERR_INVALID_EXERCISE_MODE + 3;
								return lowerlimb_sys_info;
						}

					    // Clear motor settings since each start exercise is considered a fresh start:
					    clear_lowerlimb_motors_settings(LL_motors_settings);

					    // Clear transition mode params to remove any last exercise's parameters:
					    clear_transition_mode_params();

						rx_payload_index += 1;

						switch (tcpRxData[rx_payload_index]) {
							case 0x01:
								lowerlimb_brakes_command.l_brake_disengage = true;
								lowerlimb_brakes_command.r_brake_disengage = true;
								break;
							case 0x00:
							default:
								lowerlimb_brakes_command.l_brake_disengage = false;
								lowerlimb_brakes_command.r_brake_disengage = false;
								break;
						}

						//set activity to exercise
						lowerlimb_sys_info.activity_state = EXERCISE;

						//exercise state goes to SETUP
						lowerlimb_sys_info.exercise_state = SETUP;

						//reset feedback info
						break;

					case CALIB:
						//send active calibration error message
						send_error_msg(cmd_code, ERR_CALIB_ACTIVE);
						lowerlimb_sys_info.tcp_status = ERR_CALIB_ACTIVE + 3;
						return lowerlimb_sys_info;

					case EXERCISE:
						if (lowerlimb_sys_info.exercise_state != PAUSED){
							//cover the rest with general NOK
							send_error_msg(cmd_code, ERR_GENERAL_NOK);
							lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
							return lowerlimb_sys_info;
						}

						//if paused, set to running
						lowerlimb_sys_info.exercise_state = RUNNING;
						break; // added break - GAO

					default:
						//cover the rest with general NOK
						send_error_msg(cmd_code, ERR_GENERAL_NOK);
						lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
						return lowerlimb_sys_info;
				}

				//send OK resp
				send_OK_resp(cmd_code);
				break;

			case PAUSE_EXE_CMD: //pause exercise
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				//only can paused if exercise_state is in RUNNING mode.
				if ((lowerlimb_sys_info.activity_state == EXERCISE)
						&& (lowerlimb_sys_info.exercise_state == RUNNING)) {
					lowerlimb_sys_info.exercise_state = PAUSED;
					//send OK resp
					send_OK_resp(cmd_code);
				}
				else {
					//return error message
					send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
					lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
					return lowerlimb_sys_info;
				}
				break;

			case STOP_EXE_CMD: //stop exercise
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (lowerlimb_sys_info.activity_state) {
					case IDLE:
					case CALIB:
						send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
						lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
						return lowerlimb_sys_info;
						break;
				}

				if (lowerlimb_sys_info.exercise_state == STOPPED) {
					send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
					lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
					return lowerlimb_sys_info;
					break;
				}

				//set activity to IDLE
				lowerlimb_sys_info.activity_state = IDLE;
				//set exercise_state to STOPPED
				lowerlimb_sys_info.exercise_state = STOPPED;

				//reset
				clear_lowerlimb_motors_settings(LL_motors_settings);

				//send OK resp
				send_OK_resp(cmd_code);

				break;

			case TOGGLE_SAFETY_CMD: //enable/disable safety features
				//check if rxPayload is long enough
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				lowerlimb_sys_info.safetyOFF = tcpRxData[rx_payload_index];

				//send OK resp
				send_OK_resp(cmd_code);

				break;

			case BRAKES_CMD:
				if (rxPayload != 1) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (tcpRxData[rx_payload_index]){
				  case 0x01:
						lowerlimb_brakes_command.l_brake_disengage = true;
						lowerlimb_brakes_command.r_brake_disengage = true;
						break;
					case 0x00:
					default:
						lowerlimb_brakes_command.l_brake_disengage = false;
						lowerlimb_brakes_command.r_brake_disengage = false;
				}

				//send OK resp
				send_OK_resp(cmd_code);
				break;

			/*
			case SET_OFFSET_CMD:
				if (rxPayload != (sizeof(tmp_parsed_offset_params.Fr_offset) + sizeof(tmp_parsed_offset_params.Tp_offset))) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (lowerlimb_sys_info.activity_state) {
					case IDLE:
					case CALIB:
						send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
						lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
						return lowerlimb_sys_info;
				}

				if (lowerlimb_sys_info.exercise_state == STOPPED) {
					send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
					lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
					return lowerlimb_sys_info;
				}

				//tmp_parsed_targ_params
				memcpy_msb(&tmp_parsed_offset_params.Fr_offset, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_offset_params.Fr_offset));
				rx_payload_index += sizeof(tmp_parsed_offset_params.Fr_offset);
				memcpy_msb(&tmp_parsed_offset_params.Tp_offset, &tcpRxData[rx_payload_index], sizeof(tmp_parsed_offset_params.Tp_offset));

				set_lowerlimb_offset_forces(tmp_parsed_offset_params.Fr_offset, tmp_parsed_offset_params.Tp_offset);

				//send OK resp
				send_OK_resp(cmd_code);
				break;
			*/

			case SET_CTRLPARAMS:
				if (rxPayload != (sizeof(fbgain) + sizeof(ffgain) + sizeof(compgain))) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (lowerlimb_sys_info.activity_state){
					case IDLE:
					case CALIB:
						send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
						lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
						return lowerlimb_sys_info;
				}

				if (lowerlimb_sys_info.exercise_state == STOPPED) {
					send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
					lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
					return lowerlimb_sys_info;
				}

				memcpy_msb(&fbgain, &tcpRxData[rx_payload_index], sizeof(fbgain));
				rx_payload_index += sizeof(fbgain);
				memcpy_msb(&ffgain, &tcpRxData[rx_payload_index], sizeof(ffgain));
				rx_payload_index += sizeof(ffgain);
				memcpy_msb(&compgain, &tcpRxData[rx_payload_index], sizeof(compgain));

				//Set control parameters
				configure_ctrl_params(fbgain, ffgain, compgain);

				//send OK resp
				send_OK_resp(cmd_code);
				break;

		    //////////////////////////////////////////////////////////////////////////////////////
			// SET PASSIVE TRAJECTORY CONTROL PARAMS:
			//////////////////////////////////////////////////////////////////////////////////////

		    case SET_TARG_PARAM_PTRAJCTRL_CMD:

				//check if rxPayload is long enough
				if (rxPayload != 65) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (lowerlimb_sys_info.activity_state) {
					case IDLE:
					case CALIB:
						//send exercise is not active error message
						send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
						lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
						return lowerlimb_sys_info;
						break;
					case EXERCISE:
						if (lowerlimb_sys_info.exercise_mode != PassiveTrajectoryCtrl) {
							send_error_msg(cmd_code, ERR_INVALID_EXERCISE_MODE);
							lowerlimb_sys_info.tcp_status = ERR_INVALID_EXERCISE_MODE + 3;
							return lowerlimb_sys_info;
						}
				}

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
				//send OK resp
				send_OK_resp(cmd_code);
				break;

			//////////////////////////////////////////////////////////////////////////////////////
			// SET ADMITTANCE CONTROL PARAMS:
			//////////////////////////////////////////////////////////////////////////////////////

			case SET_TARG_PARAM_ADMCTRL_CMD:

				//check if rxPayload is long enough
				if (rxPayload != 65) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (lowerlimb_sys_info.activity_state) {
				  case IDLE:
				  case CALIB:
					//send exercise is not active error message
					send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
					lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
					return lowerlimb_sys_info;
					break;
				  case EXERCISE:
					if (lowerlimb_sys_info.exercise_mode != AdmittanceCtrl) {
						send_error_msg(cmd_code, ERR_INVALID_EXERCISE_MODE);
						lowerlimb_sys_info.tcp_status = ERR_INVALID_EXERCISE_MODE + 3;
						return lowerlimb_sys_info;
					}
				}

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
				//send OK resp
				send_OK_resp(cmd_code);
				break;

		    //////////////////////////////////////////////////////////////////////////////////////
			// SET ACTIVE TRAJECTORY CONTROL PARAMS:
			//////////////////////////////////////////////////////////////////////////////////////

			case SET_TARG_PARAM_ATRAJCTRL_CMD:

				//check if rxPayload is long enough
				if (rxPayload != 65) {
					send_error_msg(cmd_code, ERR_GENERAL_NOK);
					lowerlimb_sys_info.tcp_status = ERR_GENERAL_NOK + 3;
					return lowerlimb_sys_info;
				}

				//check if system is active
				if (lowerlimb_sys_info.system_state != ON) {
					send_error_msg(cmd_code, ERR_SYSTEM_OFF);
					lowerlimb_sys_info.tcp_status = ERR_SYSTEM_OFF + 3;
					return lowerlimb_sys_info;
				}

				switch (lowerlimb_sys_info.activity_state) {
					case IDLE:
					case CALIB:
						//send exercise is not active error message
						send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
						lowerlimb_sys_info.tcp_status = ERR_EXERCISE_NOT_RUNNING + 3;
						return lowerlimb_sys_info;
					case EXERCISE:
						if (lowerlimb_sys_info.exercise_mode != ActiveTrajectoryCtrl) {
							send_error_msg(cmd_code, ERR_INVALID_EXERCISE_MODE);
							lowerlimb_sys_info.tcp_status = ERR_INVALID_EXERCISE_MODE + 3;
							return lowerlimb_sys_info;
						}
				}

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
				//send OK resp
				send_OK_resp(cmd_code);
				break;

	        case NO_CMD:
            default:
				//tx unknown command error
				send_error_msg(cmd_code, ERR_UNKNOWN);
				lowerlimb_sys_info.tcp_status = ERR_UNKNOWN + 3;
				return lowerlimb_sys_info;

		} // switch (cmd_code)
	} // if (ethernet_w5500_new_rcv_data())

	return lowerlimb_sys_info;
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
/* [] END OF FILE */
