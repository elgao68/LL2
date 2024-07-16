/////////////////////////////////////////////////////////////////////////////
//
// state_machine_ll2.c
//
// Created on: 2024.07.15
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <state_machine_ll2.h>

void
state_machine_ll2_tcp_app(uint16_t* state_fw, uint16_t* cmd_code_tcp, uint16_t* msg_code_intern, uint8_t* mode_exerc) {

	#if USE_ITM_OUT_STATE_MACH
		static uint16_t state_fw_prev   = ST_FW_SYSTEM_OFF;
		static uint8_t  mode_exerc_prev = RUNNING;
	#endif

	//////////////////////////////////////////////////////
	// State machine:
	///////////////////////////////////////////////////////

	if (*state_fw == ST_FW_SYSTEM_OFF) {
		if (*cmd_code_tcp == START_SYS_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw =  ST_FW_CONNECTING;
		}
	}

	if (*state_fw == ST_FW_CONNECTING) {
		if (*cmd_code_tcp == AUTO_CALIB_MODE_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_CALIBRATING;
		}
	}

	else if (*state_fw == ST_FW_CALIBRATING) {
		// Calibration completion reported:
		if (*msg_code_intern == CALIB_ENC_COMPLETED_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(AUTO_CALIB_MODE_CMD);

			// Change fw state:
			*state_fw = ST_FW_HOMING;
		}
	}

	else if (*state_fw == ST_FW_HOMING) {
		// Homing completion reported:
		if (*msg_code_intern == HOMING_COMPLETED_CMD) {
			// Change fw state:
			*state_fw = ST_FW_STDBY_AT_POSITION;
		}
	}

	else if (*state_fw == ST_FW_STDBY_AT_POSITION) {
		// Start / resume exercise:
		if (*cmd_code_tcp == START_RESUME_EXE_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_EXERCISE_ON;
		}
		// Stop "system":
		else if (*cmd_code_tcp == STOP_SYS_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_SYSTEM_OFF;
		}
	}

	else if (*state_fw == ST_FW_EXERCISE_ON) {
		if (*cmd_code_tcp == STOP_EXE_CMD) {
			// Exercise mode:  slow-down
			*mode_exerc = SLOWING;
		}
		// Exercise stop completion reported:
		else if (*msg_code_intern == SLOWING_COMPLETED_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(STOP_EXE_CMD);

			// Exercise mode: running (for next exercise start)
			*mode_exerc = RUNNING;

			// Change fw state:
			*state_fw = ST_FW_HOMING;
		}
	}

	//////////////////////////////////////////////////////
	// ITM Console output:
	///////////////////////////////////////////////////////

	#if USE_ITM_OUT_STATE_MACH
		if (state_fw_prev    != *state_fw ||
			mode_exerc_prev  != *mode_exerc ||
			*msg_code_intern != NO_CMD) {
				printf("   ----------------------------\n");
				printf("   state_machine_ll2_tcp_app():\n"),
				printf("   PREVIOUS:\n");
				printf("   state_fw     = [%s]\t mode_exerc = [%s] \n", STR_ST_FW[state_fw_prev - OFFS_ST_FW], MODE_EXERC_STR[mode_exerc_prev]);
				printf("   *cmd_code_tcp = [%s]\t msg_code_intern = [%s] \n", CMD_STR[*cmd_code_tcp], CMD_STR[*msg_code_intern]);
				printf("   NEW:\n");
				printf("   state_fw     = [%s]\t mode_exerc = [%s] \n", STR_ST_FW[*state_fw - OFFS_ST_FW], MODE_EXERC_STR[*mode_exerc]);
				printf("\n");
		}

		state_fw_prev   = *state_fw;
		mode_exerc_prev = *mode_exerc;
	#endif

	//////////////////////////////////////////////////////
	// Reset messages - CRITICAL:
	///////////////////////////////////////////////////////

	*cmd_code_tcp    = NO_CMD;
	*msg_code_intern = NO_CMD;
}
