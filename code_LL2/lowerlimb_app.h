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

#include <control_settings_ll2.h>
#include <lowerlimb_config.h>
#include <traj_ctrl_params_nml.h>
#include <motor_algo_ll2.h>
#include <qei_motor_drivers_LL2.h>
#include <sensors_ll2.h>

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "w5500_app.h"
#include "transition_mode.h"

///////////////////////////////////////////////////////
// Macros files:
///////////////////////////////////////////////////////

// TODO: remove at a later date:
// #include <__VALIDATE_CMD_CALIB.h>
// #include <__VALIDATE_CMD_START_EXE.h>
#include <__VALIDATE_IDLE_START_EXE.h>

#include <__SELECT_CMD_CODE_LIST.h>

///////////////////////////////////////////////////////
// CONTROL / SIMULATION SETTINGS
///////////////////////////////////////////////////////

#define USE_ITM_CMD_CHECK    1
#define SET_CTRL_PARAMETERS  1

#define TRAJ_PARAMS_VARIABLE_OFF 	0
#define TRAJ_PARAMS_VARIABLE_ON 	1

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// TYPE DEFINITIONS (OLD FIRMWARE):
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

#define LEN_STR_MAX   35
#define LEN_CMD_LIST 19

// Commands enumeration:
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

// Command strings (must match commands enumeration):
static char CMD_STR[LEN_CMD_LIST][LEN_STR_MAX] = {
	"NO_CMD",
	"SET_UNIX_CMD",
	"START_SYS_CMD",
	"STOP_SYS_CMD",
	"RESET_SYS_CMD",
	"READ_DEV_ID_CMD",
	"READ_SYS_INFO_CMD",
	"AUTO_CALIB_MODE_CMD",
	"SET_TARG_PARAM_IMPCTRL_CMD",
	"START_RESUME_EXE_CMD",
	"PAUSE_EXE_CMD",
	"STOP_EXE_CMD",
	"TOGGLE_SAFETY_CMD",
	"BRAKES_CMD",
	"SET_OFFSET_CMD",
	"SET_CTRLPARAMS",
	"SET_TARG_PARAM_PTRAJCTRL_CMD",
	"SET_TARG_PARAM_ADMCTRL_CMD",
	"SET_TARG_PARAM_ATRAJCTRL_CMD"
};

// system_state:
enum {
	SYS_OFF = 0, SYS_ON = 1
};

#define LEN_SYS_STATE 2
static char SYS_STATE_STR[LEN_SYS_STATE][LEN_STR_MAX] = {
	"SYS_OFF",
	"SYS_ON"
};

// activity_state:
enum {
	IDLE = 0, CALIB = 1, EXERCISE = 2, JOG = 3, HOMING = 4
};

#define LEN_ACTIV_STATE 5
static char ACTIV_STATE_STR[LEN_ACTIV_STATE][LEN_STR_MAX] = {
	"IDLE",
	"CALIB",
	"EXERCISE",
	"JOG",
	"HOMING"
};

// general yes or no:
enum {
	NO = 0, YES
};

// calibration result:
enum {
	FAILED = 0, PASSED
};

// exercise_state:
enum {
	STOPPED = 0, RUNNING = 1, PAUSED = 2, SETUP = 3, SLOWING = 4
};

#define LEN_EXERC_STATE 5
static char EXERC_STATE_STR[LEN_EXERC_STATE][LEN_STR_MAX] = {
	"STOPPED",
	"RUNNING",
	"PAUSED",
	"SETUP",
	"SLOWING"
};

// emergency_state:
enum {
	NORMAL = 0, WARNING = 1, ALERT
};


// brakes_state:
enum {
	ENGAGE = 0, POWERSAVE = 1, DISENGAGE = 2
};

// calibration modes:
typedef enum {
  CalibEncoders = 1,
  CalibForceSensor = 2,
  CalibEncodersFS = 3,
  AutoCalibEncodersFS = 4,
  StopCalib = 255
} Calib_protocol;


#define LEN_CALIB_MODES 5
static char CALIB_MODE_STR[LEN_CALIB_MODES][LEN_STR_MAX] = {
	" ",
	"CalibEncoders",
	"CalibForceSensor",
	"CalibEncodersFS",
	"AutoCalibEncodersFS"
};

//TCP message ID
#define CMD_MSG_TYPE							0x01
#define RESP_MSG_TYPE							0x02
#define ERROR_MSG_TYPE							0x03

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// MESSAGES & STATES: TYPE DEFINITIONS (NEW FIRMWARE)
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// App messages (internal)	MSG_APP
///////////////////////////////////////////////////////

#define	OFFS_MSG_APP		400		
#define	LEN_MSG_APP			4
enum MSG_APP {	
	MSG_APP_EXERCISE_ON		=	400	,
	MSG_APP_SELECT_EXERCISE	=	401	,
	MSG_APP_SELECT_PATIENT	=	402	,
	MSG_FW_ADJUST_EXERCISE	=	403	};

static char STR_MSG_APP[LEN_MSG_APP][LEN_STR_MAX] = {
	"MSG_APP_EXERCISE_ON"	,
	"MSG_APP_SELECT_EXERCISE"	,
	"MSG_APP_SELECT_PATIENT"	,
	"MSG_APP_ADJUST_EXERCISE"	};

///////////////////////////////////////////////////////
// Firmware messages (internal)	MSG_FW
///////////////////////////////////////////////////////

#define	OFFS_MSG_FW		500		
#define	LEN_MSG_FW		3		
enum MSG_FW {	
	MSG_FW_CALIBRATING			=	500	,
	MSG_FW_EXERCISE_ON			=	501	,
	MSG_FW_STDBY_START_POINT	=	502	};

static char STR_MSG_FW[LEN_MSG_FW][LEN_STR_MAX] = {
	"MSG_FW_CALIBRATING"	,
	"MSG_FW_EXERCISE_ON"	,
	"MSG_FW_STDBY_START_POINT"	};

///////////////////////////////////////////////////////
// TCP messages	MSG_TCP	(1st VERSION):
///////////////////////////////////////////////////////

/*
#define	OFFS_MSG_TCP		600		
#define	LEN_MSG_TCP			13

enum MSG_TCP {	
	MSG_TCP_calibrate_done		=	600	,	// dist_x, dist_y
	MSG_TCP_calibrate_robot		=	601	,
	MSG_TCP_connect_done		=	602	,
	MSG_TCP_connect_to_robot	=	603	,
	MSG_TCP_F_therapy_change	=	604	,
	MSG_TCP_go_to_exercise		=	605	,	// F_therapy
	MSG_TCP_move_to_start		=	606	,	// EX_MODE, EX_TYPE, speed_ex, point_ex_start, point_ex_end
	MSG_TCP_move_to_start_done	=	607	,
	MSG_TCP_pedal_travel		=	608	,
	MSG_TCP_robot_shutdown		=	609	,
	MSG_TCP_start_exercise		=	610	,
	MSG_TCP_stdby_start_point	=	611	,
	MSG_TCP_stop_exercise		=	612	};

static char STR_MSG_TCP[LEN_MSG_TCP][LEN_STR_MAX] = {
	"MSG_TCP_calibrate_done"	,
	"MSG_TCP_calibrate_robot"	,
	"MSG_TCP_connect_done"	,
	"MSG_TCP_connect_to_robot"	,
	"MSG_TCP_F_therapy_change"	,
	"MSG_TCP_go_to_exercise"	,
	"MSG_TCP_move_to_start"	,
	"MSG_TCP_move_to_start_done"	,
	"MSG_TCP_pedal_travel"	,
	"MSG_TCP_robot_shutdown"	,
	"MSG_TCP_start_exercise"	,
	"MSG_TCP_stdby_start_point"	,
	"MSG_TCP_stop_exercise"	};
*/

///////////////////////////////////////////////////////
// TCP messages	MSG_TCP	(2nd VERSION, based on software app command codes 06.07.2024):
///////////////////////////////////////////////////////

// Payload sizes:
#define N_PAYL_Def_MSG_TCP      		 1
#define N_PAYL_Go_To_Exercise_MSG_TCP	20
#define N_PAYL_Pedal_Travel_MSG_TCP		 8
#define N_PAYL_F_Therapy_Change_MSG_TCP	 4

#define LEN_MSG_TCP		14

enum MSG_TCP {
	NO_MSG_TCP 					= 0,
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
	Stdby_Start_Point_MSG_TCP	= 13 };

static char MSG_TCP_STR[LEN_MSG_TCP][LEN_STR_MAX] = {
	"NO_MSG_TCP",
	"_1_MSG_TCP",
	"_2_MSG_TCP",
	"Connect_To_Robot_MSG_TCP",
	"Calibrate_Robot_MSG_TCP",
	"Move_To_Start_MSG_TCP",
	"_6_MSG_TCP",
	"Go_To_Exercise_MSG_TCP",
	"Pedal_Travel_MSG_TCP",
	"Robot_Shutdown_MSG_TCP",
	"F_Therapy_Change_MSG_TCP",
	"Start_Exercise_MSG_TCP",
	"Stop_Exercise_MSG_TCP",
	"Stdby_Start_Point_MSG_TCP"
};

///////////////////////////////////////////////////////
// App states	ST_APP
///////////////////////////////////////////////////////

#define	OFFS_ST_APP		700		
#define	LEN_ST_APP		6		
enum ST_APP {	
	ST_APP_ADJUST_EXERCISE	=	700	,
	ST_APP_EXERCISE_ON		=	701	,
	ST_APP_SELECT_EXERCISE	=	702	,
	ST_APP_SELECT_PATIENT	=	703	,
	ST_APP_STDBY_CALIBRATE	=	704	,
	ST_APP_STDBY_CONNECT	=	705	};

static char STR_ST_APP[LEN_ST_APP][LEN_STR_MAX] = {
	"ST_APP_ADJUST_EXERCISE"	,
	"ST_APP_EXERCISE_ON"	,
	"ST_APP_SELECT_EXERCISE"	,
	"ST_APP_SELECT_PATIENT"	,
	"ST_APP_STDBY_CALIBRATE"	,
	"ST_APP_STDBY_CONNECT"	};
					
///////////////////////////////////////////////////////
// Firmware states	ST_FW
///////////////////////////////////////////////////////

#define	OFFS_ST_FW		800		
#define	LEN_ST_FW		5		
enum ST_FW {	
	ST_FW_ADJUST_EXERCISE	=	800	,
	ST_FW_CALIBRATING		=	801	,
	ST_FW_CONNECTING		=	802	,
	ST_FW_EXERCISE_ON		=	803	,
	ST_FW_STDBY_START_POINT	=	804	};

static char STR_ST_FW[LEN_ST_FW][LEN_STR_MAX] = {
	"ST_FW_ADJUST_EXERCISE"	,
	"ST_FW_CALIBRATING"	,
	"ST_FW_CONNECTING"	,
	"ST_FW_EXERCISE_ON"	,
	"ST_FW_STDBY_START_POINT"	};

///////////////////////////////////////////////////////
// Error codes:
///////////////////////////////////////////////////////

#define ERR_GENERAL_NOK						0x0000
#define ERR_SYSTEM_OFF						0x0001
#define ERR_EXERCISE_NOT_RUNNING			0x0003
#define ERR_INVALID_UNIX					0x0004
#define ERR_INVALID_TARGET_NUM				0x0005
#define ERR_EXERCISE_ACTIVE					0x0006
#define ERR_CALIB_ACTIVE					0x0007
#define ERR_SYSTEM_ENCRYPTED				0x0008
#define ERR_CALIBRATION_NEEDED				0x0009
#define ERR_CHECKSUM_FAILED					0x000A
#define ERR_INVALID_EXERCISE_MODE 			0x000B
#define ERR_UNKNOWN							0xBEEF

#define LEN_ERR_LIST 	13

static char ERR_STR[LEN_ERR_LIST][LEN_STR_MAX] = {
	"ERR_GENERAL_NOK"	,
	"ERR_SYSTEM_OFF"	,
	"(EMPTY)",
	"ERR_EXERCISE_NOT_RUNNING"	,
	"ERR_INVALID_UNIX"	,
	"ERR_INVALID_TARGET_NUM"	,
	"ERR_EXERCISE_ACTIVE"	,
	"ERR_CALIB_ACTIVE"	,
	"ERR_SYSTEM_ENCRYPTED"	,
	"ERR_CALIBRATION_NEEDED"	,
	"ERR_CHECKSUM_FAILED"	,
	"ERR_INVALID_EXERCISE_MODE"	,
	"ERR_UNKNOWN"
};

#define ERR_OFFSET		3

///////////////////////////////////////////////////////
// Type of operation states:
///////////////////////////////////////////////////////

#define OPS_STATUS_NORMAL_MASK			0x01
#define OPS_STATUS_EBTN_PRESSED_MASK	0x02
#define OPS_STATUS_SAMPLING_ALERT_MASK	0x04
#define OPS_STATUS_RT_VEL_ALERT_MASK	0x08

///////////////////////////////////////////////////////
// Brake states:
///////////////////////////////////////////////////////

typedef struct {
	bool l_brake_disengage;
	bool r_brake_disengage;
} lowerlimb_brakes_command_t;

#define ENGAGE_BRAKES		0
#define DISENGAGE_BRAKES	1

///////////////////////////////////////////////////////
// System info message struct
///////////////////////////////////////////////////////

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

	/* returns app status.
	1 = preample and postample failed,
	2 = incorrect msg type
	X = error code - 2, 3 = ERR_GENERAL_NOK
	*/
	uint16_t app_status;

	exercise_mode_t exercise_mode;
} lowerlimb_sys_info_t;	//System Info

///////////////////////////////////////////////////////
// Lower-limb robot system info:
///////////////////////////////////////////////////////

extern lowerlimb_sys_info_t lowerlimb_sys_info; // moved here to keep scope GLOBAL

///////////////////////////////////////////////////////
// Commanded Message Structure (state machine, 05.07.2024):
///////////////////////////////////////////////////////

typedef struct {
	uint8_t ui8Set;
	float f_x;
	float f_y;
	uint8_t ui8En_hap;
	uint8_t ui8En_main;
	float f_Kxx;
	float f_Kyy;
	float f_Kxy;
	float f_Kyx;
	float f_Bxx;
	float f_Byy;
	float f_Bxy;
	float f_Byx;
	float fGain;
	float fRot_angle;
	float resv[5];

	uint8_t index;

	uint8_t EX_MODE;
	uint8_t EX_TYPE;
	float speed_ex;
	float point_ext_start;
	float point_ext_end;
	float dist_x;
	float dist_y;
	float F_Therapy_change;
} tcp_exercise_targ_params_t;	//Set Target Parameters

///////////////////////////////////////////////////////
// Data logging message structure:
///////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////
// Message validation functions, high-level:
///////////////////////////////////////////////////////////////////////

uint8_t is_valid_rcv_data_cmd_code(uint16_t* cmd_code_ref, uint8_t ui8EBtnState, uint8_t ui8Alert, uint8_t use_software_msg_list, uint8_t overr_cmd_code_tests);

///////////////////////////////////////////////////////
// Message validation functions, low-level:
///////////////////////////////////////////////////////

static lowerlimb_brakes_command_t lowerlimb_brakes_command;

//TCP messages:
static uint8_t tcpRxData[DATA_BUF_SIZE];
static uint16_t tcpRxLen = 0;
extern ADC_HandleTypeDef hadc3;

//message preample and postample
static uint8_t PREAMP_TCP[2]  = { 0x48, 0x4D };
static uint8_t POSTAMP_TCP[2] = { 0x68, 0x6D };

///////////////////////////////////////////////////////
// Message validation functions:
///////////////////////////////////////////////////////

uint8_t is_valid_w5500_msg(uint8_t tcp_rx_data[]);
uint8_t is_valid_cmd_code_tcp_app(uint16_t cmd_code, uint8_t rxPayload, uint8_t system_state, uint8_t activity_state, uint16_t* app_status);
uint8_t is_valid_payload_size_software(uint16_t cmd_code, uint8_t rxPayload, uint16_t* app_status);

/**
 @brief: Send calibration response message
 @param[in]: actProto = current active calibration protocol
 @param[in]: percent = 0 - 100%
 @param[in]: state = 0 (0 = idle, 1 = running, 2 = passed, 3 = failed)
 */
uint8_t send_calibration_resp(uint8_t actProto, uint8_t percent, uint8_t state);
uint8_t send_error_msg(uint16_t tmp_cmd_code, uint16_t tmp_err_code);
uint8_t send_OK_resp(uint16_t tmp_cmd_code);
uint8_t send_resp_msg(uint16_t tmp_cmd_code, uint8_t tmp_payload[],	uint16_t tmp_len);

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

///////////////////////////////////////////////////////////////////////
// TCP/IP scripts (incoming messages):
///////////////////////////////////////////////////////////////////////

uint8_t lowerlimb_app_state_initialize(uint64_t init_unix, uint8_t maj_ver,
		uint8_t min_ver, uint8_t patch_ver, lowerlimb_motors_settings_t* LL_motors_settings);

// Script for reading TCP/IP messages (unified for TCP app & software (MSG_TCP):
void lowerlimb_app_onepass_ref(lowerlimb_sys_info_t* lowerlimb_sys, uint8_t ui8EBtnState, uint8_t ui8Alert, traj_ctrl_params_t* traj_ctrl_params,
		admitt_model_params_t* admitt_model_params, lowerlimb_motors_settings_t* LL_motors_settings, uint16_t* cmd_code_ref,
		uint8_t* calib_enc_on, uint8_t use_software_msg_list);

///////////////////////////////////////////////////////////////////////
// Outgoing messages:
///////////////////////////////////////////////////////////////////////

uint8_t send_lowerlimb_exercise_feedback_help(uint64_t up_time,
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

///////////////////////////////////////////////////////////////////////
// Helper functions:
///////////////////////////////////////////////////////////////////////

uint8_t send_lowerlimb_exercise_feedback(uint64_t up_time, lowerlimb_mech_readings_t* mech_readings, lowerlimb_motors_settings_t* motor_settings,
		lowerlimb_ref_kinematics_t* ref_kinematics);

void send_lowerlimb_sys_info(uint8_t tmp_resp_msg[], uint16_t cmd_code);
void reset_lowerlimb_sys_info(void);
uint8_t valid_app_status(uint8_t property, uint8_t value, uint16_t* app_status, uint16_t cmd_code, uint16_t ERR_CODE, uint16_t err_offset);
uint8_t update_unix_time(void);
void update_operation_state(uint8_t* operation_state, uint8_t ui8EBtnState, uint8_t ui8Alert);
uint8_t set_unix_time(uint64_t tmp_unix);
uint8_t set_system_on(void);
uint8_t set_system_off(void);
uint8_t set_activity_idle(void);
uint8_t set_activity_exercise(void);
uint8_t set_activity_calib(void);

uint8_t stop_exercise(lowerlimb_motors_settings_t* LL_motors_settings);
void reset_emergency_alerts(void);

bool get_l_brake_cmd(void);
bool get_r_brake_cmd(void);

void set_l_brake_status(uint8_t status);
void set_r_brake_status(uint8_t status);

void set_brakes_simple();
void set_brakes_timed(uint64_t uptime, uint64_t* brakes_next_time);

void* memcpy_msb(void *pDest, const void *pSrc, unsigned long len);

#endif

/* [] END OF FILE */
