/////////////////////////////////////////////////////////////////////////////
//
// state_machine_ll2.h
//
// Created on: 2024.07.15
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2_STATE_MACHINE_LL2_H_
#define CODE_LL2_STATE_MACHINE_LL2_H_

#include <_std_c.h>
#include <lowerlimb_app.h>
#include <_test_real_time.h>

#define USE_ITM_OUT_STATE_MACH	1

void state_machine_ll2(uint16_t* state_fw, uint16_t* cmd_code_tcp, uint16_t* msg_code_intern, uint8_t* pace_exerc);

#endif /* CODE_LL2_STATE_MACHINE_LL2_H_ */
