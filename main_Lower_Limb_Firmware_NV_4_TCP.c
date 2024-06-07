/////////////////////////////////////////////////////////////////////////////
//
// main.c - Lower_Limb_Firmware_NV_4_TCP
//
// Created on: 2024.04.24
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "stm32f4xx_hal.h"

////////////////////////////////////////////////////////////////////////////////
// User code includes:
////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "timer.h"
#include "uart_driver.h"
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "w5500_app.h"
#include "peripheral.h"

#include <_std_c.h>
#include <admitt_model_params.h>
#include <motor_algo_ll2.h>
#include <nml.h>
#include <nml_util.h>
#include <traj_ctrl_params_nml.h>
#include <lowerlimb_config.h>
// #include <lowerlimb_app.h>
#include <qei_motor_drivers_LL2.h>

#include "_test_simulation.h"
#include "_test_real_time.h"
#include "_test_scratch.h"
#include "_test_ode_int.h"
#include "_test_matr_inv.h"

////////////////////////////////////////////////////////////////////////////////
// STM32 I/O variables:
////////////////////////////////////////////////////////////////////////////////

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DAC_HandleTypeDef hdac;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
UART_HandleTypeDef huart3;

////////////////////////////////////////////////////////////////////////////////
// REAL-TIME PROCESS TIME STEP:
////////////////////////////////////////////////////////////////////////////////

#define DT_STEP_MSEC 5

////////////////////////////////////////////////////////////////////////////////
// STM32 PRIVATE FUNCTIONS - VERY IMPORTANT:
////////////////////////////////////////////////////////////////////////////////

#include <_stm32_private_funcs_c>

////////////////////////////////////////////////////////////////////////////////
// CONTROL / SIMULATION SETTINGS:
////////////////////////////////////////////////////////////////////////////////

#define _TEST_DEBUG_         0

#define _TEST_REAL_TIME      1
#define _TEST_SIMULATION	 2
#define _TEST_SCRATCH        3
#define _TEST_ODE_INT        4
#define _TEST_MATR_INV       5

#define TEST_OPTION			 _TEST_REAL_TIME

#define DT_DISP_MSEC_GUI_PARAMS		2000
#define DT_DISP_MSEC_REALTIME		1000

#define USE_ITM_OUT_GUI_PARAMS		0
#define USE_ITM_TCP_CHECK		    1
#define USE_ITM_OUT_RT_CHECK		1
#define USE_ITM_OUT_SIM				0

/////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS - DECLARATIONS - GAO
/////////////////////////////////////////////////////////////////////////////

int _write(int32_t file, uint8_t *ptr, int32_t len); // use this to send output to SWV ITM Data Console
// int __io_putchar(int ch);
void cycle_haptic_buttons();
void LED_sys_state_off();

/////////////////////////////////////////////////////////////////////////////
// MAIN():
/////////////////////////////////////////////////////////////////////////////

int main(void)
{
	uint8_t startup_status = 0;

	/////////////////////////////////////////////////////////////////////////////////////
	// Reset of all peripherals, Initializes the Flash interface and the Sys tick.
	/////////////////////////////////////////////////////////////////////////////////////

	HAL_Init();

	/////////////////////////////////////////////////////////////////////////////////////
	// Configure the system clock
	/////////////////////////////////////////////////////////////////////////////////////

	SystemClock_Config();

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize all configured peripherals:
	/////////////////////////////////////////////////////////////////////////////////////

	MX_GPIO_Init();
	MX_TIM9_Init();
	MX_USART3_UART_Init();
	MX_TIM4_Init();
	MX_SPI3_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_DAC_Init();

	/////////////////////////////////////////////////////////////////////////////////////
	// Check if you're only running "scratch" code:
	/////////////////////////////////////////////////////////////////////////////////////

	#if TEST_OPTION == _TEST_SCRATCH
		test_scratch();
		return 0;
	#endif

	/////////////////////////////////////////////////////////////////////////////////////
	//start UART sys
	/////////////////////////////////////////////////////////////////////////////////////

	/*
	uint8_t startup_status = 0;
	startup_status = uart_sys_init();

	printf("\n");
	printf("System starting up!\r\n");
	*/

	/////////////////////////////////////////////////////////////////////////////////////
	// Start 1ms timer
	/////////////////////////////////////////////////////////////////////////////////////

	startup_status = startBaseTimer();

	if (startup_status)
		printf("Base Timer init error!\r\n");
	else
		printf("Base Timer init OK\r\n");

	/////////////////////////////////////////////////////////////////////////////////////
	//start motor driver
	/////////////////////////////////////////////////////////////////////////////////////

	startup_status = motor_qei_sys_start();

	if (startup_status)
		printf("Motor PWM and QEI init error!\r\n");
	else
		printf("Motor PWM and QEI init OK\r\n");

	/////////////////////////////////////////////////////////////////////////////////////
	// Disable motors:
	/////////////////////////////////////////////////////////////////////////////////////

	motor_L_move(0, false, false);
	motor_R_move(0, false, false);

	/////////////////////////////////////////////////////////////////////////////////////
	// Set up Ethernet:
	/////////////////////////////////////////////////////////////////////////////////////

	Ethernet_Reset(true);
	HAL_Delay(1000);
	Ethernet_Reset(false);
	HAL_Delay(1000);
	set_ethernet_w5500_mac(0x00, 0x0a, 0xdc, 0xab, 0xcd, 0xef);
	ethernet_w5500_sys_init();

	/////////////////////////////////////////////////////////////////////////////////////
	// Start DAC ports:
	/////////////////////////////////////////////////////////////////////////////////////

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	/////////////////////////////////////////////////////////////////////////////////////
	// Cycle LEDs:
	/////////////////////////////////////////////////////////////////////////////////////

	Cycle_LED_Init();
	LED_sys_state_off();

	/////////////////////////////////////////////////////////////////////////////////////
	// Launch test script:
	/////////////////////////////////////////////////////////////////////////////////////

	#if TEST_OPTION == _TEST_REAL_TIME
		test_real_time(&hadc1, &hadc3);
	#elif TEST_OPTION == _TEST_SIMULATION
		test_simulation();
	#elif TEST_OPTION == _TEST_ODE_INT
		test_ode_int();
	#elif TEST_OPTION == _TEST_MATR_INV
		test_matr_inv();
	#endif
}

/////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS - DEFINITIONS - GAO
/////////////////////////////////////////////////////////////////////////////

int
_write(int32_t file, uint8_t *ptr, int32_t len) {
	for(int i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}

/*
int
__io_putchar(int ch) {
	// Write character to ITM ch.0
	ITM_SendChar(ch);
	return(ch);
}
*/

void
cycle_haptic_buttons() {
	if (Read_Haptic_Button()) {
		Left_LED_function(Blue);
		Right_LED_function(Blue);
	}
	else {
		Left_LED_function(Red);
		Right_LED_function(Red);
	}
}

void
LED_sys_state_off() {
	Left_LED_function(Yellow);
	Right_LED_function(Yellow);
}
