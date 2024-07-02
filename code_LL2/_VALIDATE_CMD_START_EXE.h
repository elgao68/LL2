/////////////////////////////////////////////////////////////////////////////
//
// _VALIDATE_CMD_START_EXE.h
//
// Created on: 2024.07.01
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2__VALIDATE_CMD_START_EXE_H_
#define CODE_LL2__VALIDATE_CMD_START_EXE_H_

#define _VALIDATE_CMD_START_EXE \
if (lowerlimb_sys_info.activity_state == EXERCISE) { \
	if (!valid_app_status(lowerlimb_sys_info.exercise_state, PAUSED, \
		&lowerlimb_sys_info.app_status, cmd_code, ERR_GENERAL_NOK, ERR_OFFSET)) \
			return lowerlimb_sys_info; \
	lowerlimb_sys_info.exercise_state = RUNNING; \
} \
else if (lowerlimb_sys_info.activity_state == CALIB) { \
	printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_CALIB_ACTIVE]); \
	send_error_msg(cmd_code, ERR_CALIB_ACTIVE); \
	lowerlimb_sys_info.app_status = ERR_CALIB_ACTIVE + 3; \
	return lowerlimb_sys_info; \
} \
else { \
	printf("   lowerlimb_app_state_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_GENERAL_NOK]); \
	send_error_msg(cmd_code, ERR_GENERAL_NOK); \
	lowerlimb_sys_info.app_status = ERR_GENERAL_NOK + 3; \
	return lowerlimb_sys_info; \
}

#endif /* CODE_LL2__VALIDATE_CMD_START_EXE_H_ */
