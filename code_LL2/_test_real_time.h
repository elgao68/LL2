/////////////////////////////////////////////////////////////////////////////
//
//  _test_real_time.h
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2__TEST_REAL_TIME_H_
#define CODE_LL2__TEST_REAL_TIME_H_

#include <_std_c.h>
#include <control_settings_ll2.h>
#include <lowerlimb_app.h>
#include <motor_algo_ll2.h>
#include <nml.h>
#include <nml_util.h>
#include <traj_ctrl_params_nml.h>
#include <string.h>
#include <linear_systems.h>
#include <state_machine_ll2.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "timer.h"
#include "uart_driver.h"
#include "qei_motor_drivers_LL2.h"
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "w5500_app.h"
#include "peripheral.h"
#include "socket.h"

////////////////////////////////////////////////////////////////////////////////
// Program settings:
////////////////////////////////////////////////////////////////////////////////

// Switch robot mode / standalone board mode - CRITICAL:
#define MODE_STANDALONE_BOARD		1

// TCP messages list to use (TCP app: 0 / software app: 1) - CRITICAL:
#define USE_SOFTWARE_MSG_LIST		1

// Dynamic system mode: unconstrained / constrained:
#define ADMITT_MODEL_CONSTR_ON		1
#define ADMITT_MODEL_CONSTR_OFF		0

// Control overrides:
#define OVERR_DYN_PARAMS_RT			1
#define OVERR_FORCE_SENSORS_CALIB	0
#define OVERR_BRAKES				0
#define OVERR_DATA_FEEDBACK			0

// ITM Data Console switches:
#define USE_ITM_OUT_RT_CHECK_CTRL		1
#define USE_ITM_OUT_RT_CHECK_CTRL_LONG	0
#define USE_ITM_OUT_RT_CHECK_STATES		0
#define USE_ITM_OUT_UPTIME_CHECK		0
#define USE_ITM_OUT_GUI_PARAMS			0
#define USE_ITM_TCP_CHECK		    	0

// Assorted timers:
#define DT_EXPIRE_MSEC				 1000
#define DT_DISP_MSEC_REALTIME		 2000
#define DT_DISP_MSEC_GUI_PARAMS		 2000

// TCP ethernet related - Demo Firmware Version:
#define VER_H		0x00
#define VER_L		0x00
#define VER_P		0x00

////////////////////////////////////////////////////////////////////////////////
// Define macros:
////////////////////////////////////////////////////////////////////////////////

#if USE_SOFTWARE_MSG_LIST
	#define __CMD_CODE_STR(cmd_code) \
		MSG_TCP_STR[cmd_code]
#else
	#define __CMD_CODE_STR(cmd_code) \
		CMD_STR[cmd_code]
#endif

////////////////////////////////////////////////////////////////////////////////
// Control variables:
////////////////////////////////////////////////////////////////////////////////

static uint64_t algo_nextTime = 0;
static uint64_t brakes_nextTime = 0;
static uint64_t expire_nextTime = 0;
static uint8_t prev_fifo_size = 0;

// ADC:
static uint32_t dum_force_end_in_x = 0;
static uint32_t dum_force_end_in_y = 0;
static uint32_t current_sensor_L = 0;
static uint32_t current_sensor_R = 0;

////////////////////////////////////////////////////////////////////////////////
// Declare tester functions (see main.c)::
////////////////////////////////////////////////////////////////////////////////

void test_real_time_onepass_control(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3);
void test_real_time_onepass_software(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3);
void test_real_time_statemach_control(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3);

////////////////////////////////////////////////////////////////////////////////
// Declare helper functions (see main.c):
////////////////////////////////////////////////////////////////////////////////

void cycle_haptic_buttons();
void LED_sys_state_off();

#endif /* CODE_LL2__TEST_REAL_TIME_H_ */
