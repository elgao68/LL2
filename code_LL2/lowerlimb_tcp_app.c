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

///////////////////////////////////////////////////////////////////////
// Messaging functions:
///////////////////////////////////////////////////////////////////////

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

	if (percent == 100) {
		//reset acitivity to IDLE
		set_activity_idle();
	}
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
send_lowerlimb_exercise_feedback(uint64_t up_time,
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
set_lowerlimb_exercise_feedback(uint64_t up_time, lowerlimb_mech_readings_t* mech_readings, lowerlimb_motors_settings_t* motor_settings,
		lowerlimb_ref_kinematics_t* ref_kinematics) {

	float FORCE_END_MAGN = 0.0; // don't need to maintain this

	return send_lowerlimb_exercise_feedback(
		up_time,
		mech_readings->coord.x,
		mech_readings->coord.y,
		mech_readings->left.qei_count,
		mech_readings->right.qei_count,
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
