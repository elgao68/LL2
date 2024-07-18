/////////////////////////////////////////////////////////////////////////////
//
// __SELECT_CMD_CODE_LIST.h
//
// Created on: 2024.07.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2___SELECT_CMD_CODE_LIST_H_
#define CODE_LL2___SELECT_CMD_CODE_LIST_H_

// TODO: complete pending command codes (TCP app):

#define __SELECT_CMD_CODE_LIST(USE_SOFTWARE_MSG_LIST) \
	uint8_t 	_START_SYSTEM;	\
	uint8_t 	_BRAKES_ON_OFF;	\
	uint8_t 	_CALIBRATE;	\
	uint8_t 	_START_EXERCISE;	\
	uint8_t 	_STOP_EXERCISE;	\
	uint8_t 	_STOP_SYSTEM;	\
	uint8_t 	_MOVE_TO_START;	\
	uint8_t 	_GO_TO_EXERCISE;	\
	uint8_t 	_PEDAL_TRAVEL;	\
	uint8_t 	_FORCE_THERAPY_CHANGE;	\
	uint8_t 	_STDBY_START_POINT;	\
	\
	if (USE_SOFTWARE_MSG_LIST) { \
		_START_SYSTEM = Connect_To_Robot_MSG_TCP;	\
		_BRAKES_ON_OFF = 0;	\
		_CALIBRATE = Calibrate_Robot_MSG_TCP;	\
		_START_EXERCISE = Start_Exercise_MSG_TCP;	\
		_STOP_EXERCISE = Stop_Exercise_MSG_TCP;	\
		_STOP_SYSTEM = Robot_Shutdown_MSG_TCP;	\
		_MOVE_TO_START = Move_To_Start_MSG_TCP;	\
		_GO_TO_EXERCISE = Go_To_Exercise_MSG_TCP;	\
		_PEDAL_TRAVEL = Pedal_Travel_MSG_TCP;	\
		_FORCE_THERAPY_CHANGE = F_Therapy_Change_MSG_TCP;	\
		_STDBY_START_POINT = Stdby_Start_Point_MSG_TCP;	\
	} \
	else { \
		_START_SYSTEM = START_SYS_CMD;	\
		_BRAKES_ON_OFF = BRAKES_CMD;	\
		_CALIBRATE = AUTO_CALIB_MODE_CMD;	\
		_START_EXERCISE = START_EXERCISE_CMD;	\
		_STOP_EXERCISE = STOP_EXERCISE_CMD;	\
		_STOP_SYSTEM = STOP_SYS_CMD;	\
		_MOVE_TO_START        = 0;	\
		_GO_TO_EXERCISE       = 0;	\
		_PEDAL_TRAVEL         = 0;	\
		_FORCE_THERAPY_CHANGE = 0;	\
		_STDBY_START_POINT    = 0;	\
	}

#endif /* CODE_LL2___SELECT_CMD_CODE_LIST_H_ */
