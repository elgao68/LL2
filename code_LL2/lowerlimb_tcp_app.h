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
 * Hman TCP App code
 * ========================================
 */

#ifndef _LOWERLIMB_TCP_APP_H_
#define _LOWERLIMB_TCP_APP_H_

#include <admitt_model_params.h>
#include <lowerlimb_config.h>
#include <traj_ctrl_params_nml.h>
#include <motor_algo_ll2.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "common.h"

///////////////////////////////////////////////////////
// CONTROL / SIMULATION SETTINGS - GAO
///////////////////////////////////////////////////////

#define SET_CTRL_PARAMETERS  1

///////////////////////////////////////////////////////
// TYPE DEFINITIONS:
///////////////////////////////////////////////////////

enum {
	NO_CMD = 0,
	SET_UNIX_CMD = 1,
	START_SYS_CMD = 2,
	STOP_SYS_CMD = 3,
	RESET_SYS_CMD = 4,
	READ_DEV_ID_CMD = 5,
	READ_SYS_INFO_CMD = 6,
	AUTO_CALIB_MODE_CMD = 7,
	SET_TARG_PARAM_IMPCTRL_CMD = 8,
	START_RESUME_EXE_CMD = 9,
	PAUSE_EXE_CMD = 10,
	STOP_EXE_CMD = 11,
	TOGGLE_SAFETY_CMD = 12,
	BRAKES_CMD = 13,
	SET_OFFSET_CMD = 14,
	SET_CTRLPARAMS = 15,
	SET_TARG_PARAM_PTRAJCTRL_CMD = 16,
	SET_TARG_PARAM_ADMCTRL_CMD = 17,
	SET_TARG_PARAM_ATRAJCTRL_CMD = 18,
};
//command

enum {
	OFF = 0, ON
};
//system_state

enum {
	IDLE = 0, CALIB = 1, EXERCISE
};
//activity_state

enum {
	NO = 0, YES
};
//general yes or no

enum {
	FAILED = 0, PASSED
};
//calibration result

enum {
	STOPPED = 0, RUNNING = 1, PAUSED = 2, SETUP = 3,
};
//exercise_state

enum {
	NORMAL = 0, WARNING = 1, ALERT
};
//emergency_state

enum {
	ENGAGE = 0, POWERSAVE = 1, DISENGAGE = 2
};
//brakes_state

typedef enum {
  CalibEncoders = 1,
  CalibForceSensor = 2,
  CalibEncodersFS = 3,
  AutoCalibEncodersFS = 4,
  StopCalib = 255
} Calib_protocol;

//TCP message ID
#define CMD_MSG_TYPE							0x01
#define RESP_MSG_TYPE							0x02
#define ERROR_MSG_TYPE						0x03

//Error code
#define ERR_GENERAL_NOK						0x0000
#define ERR_SYSTEM_OFF						0x0001
#define ERR_EXERCISE_NOT_RUNNING	0x0003
#define ERR_INVALID_UNIX					0x0004
#define ERR_INVALID_TARGET_NUM		0x0005
#define ERR_EXERCISE_ACTIVE				0x0006
#define ERR_CALIB_ACTIVE					0x0007
#define ERR_SYSTEM_ENCRYPTED			0x0008
#define ERR_CALIBRATION_NEEDED		0x0009
#define ERR_CHECKSUM_FAILED				0x000A
#define ERR_INVALID_EXERCISE_MODE 0x000B
#define ERR_UNKNOWN								0xBEEF

//type of operation states
#define OPS_STATUS_NORMAL_MASK			0x01
#define OPS_STATUS_EBTN_PRESSED_MASK	0x02
#define OPS_STATUS_SAMPLING_ALERT_MASK	0x04
#define OPS_STATUS_RT_VEL_ALERT_MASK	0x08

typedef struct {
	bool l_brake_disengage;
	bool r_brake_disengage;
} lowerlimb_brakes_command_t;

///////////////////////////////////////////////////////////////////////
// System info message struct - GAO
///////////////////////////////////////////////////////////////////////

typedef struct {
	uint8_t device_id[14];
	uint64_t unix;
	uint8_t system_state;		//system ON or OFF
	uint8_t exercise_state;		//stopped, running, paused or in setup
	uint8_t calibrate_state;	//running or not running
	uint8_t operation_state;	//bit mask of events
	uint8_t fw_maj_ver;
	uint8_t fw_min_ver;
	uint8_t fw_patch_ver;
	uint8_t safetyOFF;			//0 = safety on, 1 = safety off
	uint8_t l_brake_status;
	uint8_t r_brake_status;
	float f_x;
	float f_y;

	uint8_t activity_state;		//idle, exercise or calib

	uint8_t isCalibrated;		//0 = not calibrated. 1 = calibrated
	uint8_t calib_prot_req; 	//which protocol was requested

	/*returns tcp app status.
	 1 = preample and postample failed,
	 2 = incorrect msg type
	 X = error code - 2, 3 = ERR_GENERAL_NOK*/
	uint16_t tcp_status;

	exercise_mode_t exercise_mode;
} lowerlimb_sys_info_t;	//System Info

///////////////////////////////////////////////////////////////////////
// Data logging message struct - GAO
///////////////////////////////////////////////////////////////////////

typedef struct {
	uint32_t timestamp;
	float f_x;
	float f_y;
	int32_t fQei_L;
	int32_t fQei_R;
	float fVel_X;
	float fVel_Y;
	float fVolt_L;
	float fVolt_R;
	float fCs_L;
	float fCs_R;
	float fFs_X;
	float fFs_Y;
	float fCmd_Fx;
	float fCmd_Fy;
	float fCmd_Fr;
	float fResv;
	uint8_t ui8OpsStatus;
	uint8_t l_brake_status;
	uint8_t r_brake_status;

} lowerlimb_exercise_feedback_params_t;

void* memcpy_msb(void *pDest, const void *pSrc, unsigned long len);

/**
 @brief: Send calibration response message
 @param[in]: actProto = current active calibration protocol
 @param[in]: percent = 0 - 100%
 @param[in]: state = 0 (0 = idle, 1 = running, 2 = passed, 3 = failed)
 */
uint8_t send_calibration_resp(uint8_t actProto, uint8_t percent, uint8_t state);

/**
 @brief: reset the lowerlimb_exercise_feedback_info
 @param[in]: f_x = current X-coordinate
 @param[in]: f_y = current Y-coordinate
 @param[in]: fQei_L = left QEI count
 @param[in]: fQei_R = right QEI count
 @param[in]: fVel_X = X-axis velocity (m/s)
 @param[in]: fVel_Y = Y-axis velocity (m/s)
 @param[in]: fVolt_L = left motor voltage (V)
 @param[in]: fVolt_R = right motor voltage (V)
 @param[in]: fCs_L = left current sense (A)
 @param[in]: fCs_R = right current sense (A)
 @param[in]: f_force = RMS force on X-Y axes
 @param[in]: ui8A0 = additional input 0
 @param[in]: ui8A1 = additional input 1
 @retval: 0 = success, 1 = failed
 */

uint8_t
set_LL_exercise_feedback_help(uint64_t up_time, lowerlimb_mech_readings_t* mech_readings, lowerlimb_motors_settings_t* motor_settings,
		lowerlimb_ref_kinematics_t* ref_kinematics);

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
								float fRefPhase, float fRefFreq);

/**
 @brief: stop exercise
 @retval: 0 = success, 1 = failed
 */
uint8_t stop_exercise(lowerlimb_motors_settings_t* LL_motors_settings);

/**
 @brief: Start in the APP.
 @retval: 0 = success, 1 = failed
 */
uint8_t lowerlimb_tcp_init_app_state(uint64_t init_unix, uint8_t maj_ver,
		uint8_t min_ver, uint8_t patch_ver, lowerlimb_motors_settings_t* LL_motors_settings);

/**
 @brief: Run in the main().
 @param[in]: ui8EBtnState = emergency button (1 = normal, 0 = asserted)
 @param[in]: ui8Alert = alert msg from motor algo (0 = no alert, 1 = sampling alert, 2 = RT vel alert)
 @retval: 0 = success, 1 = failed
 */
lowerlimb_sys_info_t lowerlimb_tcp_app_state(uint8_t ui8EBtnState, uint8_t ui8Alert,
		traj_ctrl_params_t* traj_ctrl_params, admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings);

/**
 @brief: reset all alerts and set normal state.
 */
void reset_emergency_alerts(void);

bool get_l_brake_cmd(void);
bool get_r_brake_cmd(void);

void set_l_brake_status(uint8_t status);
void set_r_brake_status(uint8_t status);
#endif

/* [] END OF FILE */
