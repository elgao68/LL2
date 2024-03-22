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
#include <lowerlimb_tcp_app.h>
#include <motor_algo_ll2.h>
#include <nml.h>
#include <nml_util.h>
#include <traj_ctrl_params_nml.h>

#include <string.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "timer.h"
#include "uart_driver.h"
#include "qei_motor_drivers.h"
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "w5500_app.h"
#include "peripheral.h"

////////////////////////////////////////////////////////////////////////////////
// USER CODE BEGIN PV
////////////////////////////////////////////////////////////////////////////////

// TCP ethernet related - Demo Firmware Version

#define VER_H		0x00
#define VER_L		0x00
#define VER_P		0x00

#define INTERVAL_5MS		5
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
// static uint64_t R_brakes_powersavetimer = 0;
// static uint64_t P_brakes_powersavetimer = 0;

// Dynamic system mode: unconstrained / constrained:
#define USE_ADMITT_MODEL_CONSTR_RT	0
#define OVERR_DYN_PARAMS_RT			1

#define DT_EXPIRE_MSEC		 	1000

// Force sensors override:
#define OVERR_FORCE_SENSORS_CALIB_1		1 // ensure calibration mode 1 is selected

////////////////////////////////////////////////////////////////////////////////
// Declare tester function:
////////////////////////////////////////////////////////////////////////////////

void test_real_time(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3);

#endif /* CODE_LL2__TEST_REAL_TIME_H_ */
