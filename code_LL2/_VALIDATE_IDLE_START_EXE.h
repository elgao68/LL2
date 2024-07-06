/////////////////////////////////////////////////////////////////////////////
//
// _VALIDATE_IDLE_START_EXE.h
//
// Created on: 2024.07.01
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2__VALIDATE_IDLE_START_EXE_H_
#define CODE_LL2__VALIDATE_IDLE_START_EXE_H_

// NOTE: removed lowerlimb_sys_info.exercise_mode != AdmittanceCtrl

#define _VALIDATE_IDLE_START_EXE \
\
if (!valid_app_status(lowerlimb_sys_info.isCalibrated, NO, &lowerlimb_sys_info.app_status, cmd_code, ERR_CALIBRATION_NEEDED, ERR_OFFSET)) return lowerlimb_sys_info; \
\
lowerlimb_sys_info.exercise_mode = (exercise_mode_t)tcpRxData[rx_payload_index]; \
rx_payload_index += 1; \
\
if (lowerlimb_sys_info.exercise_mode != PassiveTrajectoryCtrl && \
	lowerlimb_sys_info.exercise_mode != ActiveTrajectoryCtrl)  { \
		printf("   lowerlimb_sys_info.exercise_mode = [%s]\n", EXERC_MODE_STR[lowerlimb_sys_info.exercise_mode]); \
		printf("   lowerlimb_app_onepass_tcpip() ERROR: cmd [%s] generated error [%s]\n\n", CMD_STR[cmd_code], ERR_STR[ERR_INVALID_EXERCISE_MODE]); \
		send_error_msg(cmd_code, ERR_INVALID_EXERCISE_MODE); \
		lowerlimb_sys_info.app_status = ERR_INVALID_EXERCISE_MODE + 3; \
		return lowerlimb_sys_info; \
} \
\
clear_lowerlimb_motors_settings(LL_motors_settings); \
clear_transition_mode_params(); \
rx_payload_index += 1;

#endif /* CODE_LL2__VALIDATE_IDLE_START_EXE_H_ */
