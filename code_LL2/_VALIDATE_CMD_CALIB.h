/////////////////////////////////////////////////////////////////////////////
//
// _VALIDATE_CMD_CALIB.h
//
// Created on: 2024.07.01
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2__VALIDATE_CMD_CALIB_H_
#define CODE_LL2__VALIDATE_CMD_CALIB_H_

#define _VALIDATE_CMD_CALIB \
if (lowerlimb_sys_info.activity_state == EXERCISE) { \
	printf("   lowerlimb_app_onepass_tcp_app_ref() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_EXERCISE_ACTIVE]); \
	send_error_msg(cmd_code, ERR_EXERCISE_ACTIVE); \
	lowerlimb_sys_info.app_status = ERR_EXERCISE_ACTIVE + 3; \
	return lowerlimb_sys_info; \
} \
else if (lowerlimb_sys_info.activity_state == CALIB) { \
	if (tcpRxData[rx_payload_index] != 255) { \
		printf("   lowerlimb_app_onepass_tcp_app_ref() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_CALIB_ACTIVE]); \
		send_error_msg(cmd_code, ERR_CALIB_ACTIVE); \
		lowerlimb_sys_info.app_status = ERR_CALIB_ACTIVE + 3; \
		return lowerlimb_sys_info; \
	} \
} \
else { \
	printf("   lowerlimb_app_onepass_tcp_app_ref() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_GENERAL_NOK]); \
	send_error_msg(cmd_code, ERR_GENERAL_NOK); \
	lowerlimb_sys_info.app_status = ERR_GENERAL_NOK + 3; \
	return lowerlimb_sys_info; \
}

#endif /* CODE_LL2__VALIDATE_CMD_CALIB_H_ */
