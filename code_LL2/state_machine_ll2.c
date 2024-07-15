/////////////////////////////////////////////////////////////////////////////
//
// state_machine_ll2.c
//
// Created on: 2024.07.15
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "state_machine_ll2.h"

void
state_machine_ll2_tcp_app(uint16_t* state_fw, uint16_t cmd_code_tcp, uint16_t* msg_code_intern, uint8_t* mode_exerc) {

	if (*state_fw == ST_FW_SYSTEM_OFF) {
		if (cmd_code_tcp == START_SYS_CMD) {
			// Response to cmd_code_tcp:
			send_OK_resp(cmd_code_tcp);

			// Change fw state:
			*state_fw =  ST_FW_CONNECTING;
		}
	}

	if (*state_fw == ST_FW_CONNECTING) {
		if (cmd_code_tcp == AUTO_CALIB_MODE_CMD) {
			// Response to cmd_code_tcp:
			send_OK_resp(cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_CALIBRATING;
		}
	}

	else if (*state_fw == ST_FW_CALIBRATING) {
		// Calibration completion reported:
		if (*msg_code_intern == CALIB_ENC_COMPLETED_CMD) {

			// Reset internal command code:
			*msg_code_intern = NO_CMD;

			// Response to cmd_code_tcp:
			send_OK_resp(AUTO_CALIB_MODE_CMD);

			// Change fw state:
			*state_fw = ST_FW_HOMING;
		}
	}

	else if (*state_fw == ST_FW_HOMING) {
		// Homing completion reported:
		if (*msg_code_intern == HOMING_COMPLETED_CMD) {

			// Reset internal command code:
			*msg_code_intern = NO_CMD;

			// Change fw state:
			*state_fw = ST_FW_STDBY_AT_POSITION;
		}
	}

	else if (*state_fw == ST_FW_STDBY_AT_POSITION) {
		// Start / resume exercise:
		if (cmd_code_tcp == START_RESUME_EXE_CMD) {
			// Response to cmd_code_tcp:
			send_OK_resp(cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_EXERCISE_ON;
		}
		// Stop "system":
		else if (cmd_code_tcp == STOP_SYS_CMD) {
			// Response to cmd_code_tcp:
			send_OK_resp(cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_SYSTEM_OFF;
		}
	}

	else if (*state_fw == ST_FW_EXERCISE_ON) {
		if (cmd_code_tcp == STOP_EXE_CMD)
			// Exercise mode:  slow-down
			*mode_exerc = SLOWING;

		// Exercise stop completion reported:
		else if (*msg_code_intern == SLOWING_COMPLETED_CMD) {
			// Reset internal command code:
			*msg_code_intern = NO_CMD;

			// Response to cmd_code_tcp:
			send_OK_resp(STOP_EXE_CMD);

			// Exercise mode: running (for next exercise start)
			*mode_exerc = RUNNING;

			// Change fw state:
			*state_fw = ST_FW_HOMING;
		}
	}
}
