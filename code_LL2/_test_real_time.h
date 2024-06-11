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
#include <admitt_model_params.h>
#include <lowerlimb_app.h>
#include <motor_algo_ll2.h>
#include <nml.h>
#include <nml_util.h>
#include <traj_ctrl_params_nml.h>
#include <string.h>

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
// USER CODE BEGIN PV
////////////////////////////////////////////////////////////////////////////////

// TCP ethernet related - Demo Firmware Version:
#define VER_H		0x00
#define VER_L		0x00
#define VER_P		0x00

// uint64_t ethernet_test_nextTime = 0;

static uint64_t algo_nextTime = 0;
static uint64_t brakes_nextTime = 0;
static lowerlimb_sys_info_t LL_sys_info;
static uint64_t expire_nextTime = 0;
static uint8_t prev_fifo_size = 0;

// ADC
static uint32_t dum_force_end_in_x = 0;
static uint32_t dum_force_end_in_y = 0;
static uint32_t current_sensor_L = 0;
static uint32_t current_sensor_R = 0;

// Motor driver feedback related:
// static uint8_t tcpTxData[292];
// static uint64_t L_brakes_powersavetimer = 0;
// static uint64_t R_brakes_powersavetimer = 0;

// Dynamic system mode: unconstrained / constrained:
#define USE_ADMITT_MODEL_CONSTR_RT	1
#define OVERR_DYN_PARAMS_RT			1
#define OVERR_FORCE_SENSORS_CALIB	0
#define USE_APP_STATE_TEMPLATE		1

// Motor torque activation (VERY IMPORTANT):
#define MOTOR_TORQUE_ON				0

// Assorted timers:
#define DT_EXPIRE_MSEC				1000
#define DT_DISP_MSEC_REALTIME		1000
#define DT_DISP_MSEC_GUI_PARAMS		2000

// ITM Data Console switches:
#define USE_ITM_OUT_RT_CHECK		1
#define USE_ITM_OUT_GUI_PARAMS		0
#define USE_ITM_TCP_CHECK		    0

////////////////////////////////////////////////////////////////////////////////
// Declare helper functions (see main.c):
////////////////////////////////////////////////////////////////////////////////

void cycle_haptic_buttons();

void LED_sys_state_off();

////////////////////////////////////////////////////////////////////////////////
// Declare tester function:
////////////////////////////////////////////////////////////////////////////////

void test_real_time(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3);

#endif /* CODE_LL2__TEST_REAL_TIME_H_ */
