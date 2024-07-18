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
state_machine_ll2(uint16_t* state_fw, uint16_t* cmd_code_tcp, uint16_t* msg_code_intern, uint8_t* pace_exerc) {

	//////////////////////////////////////////////////////
	// Select TCP command codes list (CRITICAL):
	//////////////////////////////////////////////////////

	// NOTE use of global variable USE_SOFTWARE_MSG_LIST:
	__SELECT_CMD_CODE_LIST(USE_SOFTWARE_MSG_LIST)

	//////////////////////////////////////////////////////
	// ITM Console output:
	///////////////////////////////////////////////////////

	#if USE_ITM_OUT_STATE_MACH
		static uint16_t state_fw_prev   = ST_FW_SYSTEM_OFF;
		static uint8_t  pace_exerc_prev = RUNNING;
	#endif

	//////////////////////////////////////////////////////
	// State machine:
	///////////////////////////////////////////////////////

	// General command codes - unused yet:
	/*
	_BRAKES_ON_OFF
	_MOVE_TO_START
	_GO_TO_EXERCISE
	_PEDAL_TRAVEL
	_FORCE_THERAPY_CHANGE
	_STDBY_START_POINT
	*/

	if (*state_fw == ST_FW_SYSTEM_OFF) {
		if (*cmd_code_tcp == _START_SYSTEM) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw =  ST_FW_CONNECTING;
		}
	}

	if (*state_fw == ST_FW_CONNECTING) {
		if (*cmd_code_tcp == _CALIBRATE) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_CALIBRATE_AND_HOME;
		}
		// Stop "system":
		else if (*cmd_code_tcp == _STOP_SYSTEM) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_SYSTEM_OFF;
		}
	}

	else if (*state_fw == ST_FW_CALIBRATE_AND_HOME) {
		// Calibration completion reported:
		if (*msg_code_intern == CALIB_ENC_COMPLETED_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(_CALIBRATE);

			// Change fw state:
			*state_fw = ST_FW_STDBY_AT_POSITION;
		}
	}

	else if (*state_fw == ST_FW_STDBY_AT_POSITION) {
		// Start / resume exercise:
		if (*cmd_code_tcp == _START_EXERCISE) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_EXERCISE_ON;
		}
		// Stop "system":
		else if (*cmd_code_tcp == _STOP_SYSTEM) {
			// Response to *cmd_code_tcp:
			send_OK_resp(*cmd_code_tcp);

			// Change fw state:
			*state_fw = ST_FW_SYSTEM_OFF;
		}
	}

	else if (*state_fw == ST_FW_EXERCISE_ON) {
		if (*cmd_code_tcp == _STOP_EXERCISE) {
			// Exercise mode:  slow-down
			*pace_exerc = SLOWING;
		}
		// Exercise stop completion reported:
		else if (*msg_code_intern == SLOWING_COMPLETED_CMD) {
			// Response to *cmd_code_tcp:
			send_OK_resp(_STOP_EXERCISE);

			// Exercise mode: running (for next exercise start)
			*pace_exerc = RUNNING;

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

	//////////////////////////////////////////////////////
	// ITM Console output:
	///////////////////////////////////////////////////////

	#if USE_ITM_OUT_STATE_MACH
		if (state_fw_prev    != *state_fw   ||
			pace_exerc_prev  != *pace_exerc ||
			*msg_code_intern != NO_CMD) {
				printf("   ----------------------------\n");
				printf("   state_machine_ll2():\n"),
				printf("   PREVIOUS:\n");
				printf("   state_fw = [%s]\t pace_exerc = [%s] \n", STR_ST_FW[state_fw_prev - OFFS_ST_FW], PACE_EXERC_STR[pace_exerc_prev]);
				printf("   cmd_code_tcp = [%s]\t msg_code_intern = [%s] \n", __CMD_CODE_STR(*cmd_code_tcp), __CMD_CODE_STR(*msg_code_intern));
				printf("   NEW:\n");
				printf("   state_fw = [%s]\t pace_exerc = [%s] \n", STR_ST_FW[*state_fw - OFFS_ST_FW], PACE_EXERC_STR[*pace_exerc]);
				printf("\n");
		}

		state_fw_prev   = *state_fw;
		pace_exerc_prev = *pace_exerc;
	#endif

	//////////////////////////////////////////////////////
	// Reset messages - CRITICAL:
	///////////////////////////////////////////////////////

	*cmd_code_tcp    = NO_CMD;
	*msg_code_intern = NO_CMD;
}
