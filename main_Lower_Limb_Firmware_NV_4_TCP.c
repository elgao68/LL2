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

/* USER CODE BEGIN Includes */
#include <string.h>
#include "timer.h"
#include "uart_driver.h"
#include "qei_motor_drivers.h"
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "w5500_app.h"
#include "distal_tcp_app.h"
#include "motor_algo.h"
#include "distal_calib_protocols.h"
#include "peripheral.h"
/* USER CODE END Includes */

////////////////////////////////////////////////////////////////////////////////
// Demo Firmware Version:
////////////////////////////////////////////////////////////////////////////////

#define VER_H		0x00
#define VER_L		0x00
#define VER_P		0x00

////////////////////////////////////////////////////////////////////////////////
// Private variables:
////////////////////////////////////////////////////////////////////////////////

#define DT_STEP_MSEC 5

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

uint64_t ethernet_test_nextTime = 0;
static uint64_t algo_nextTime = 0;

static uint64_t brakes_nextTime = 0;
static uint64_t uart_output_nextTime = 0;
static distal_motors_settings_t current_motors_settings;
static distal_mech_readings_t current_mech_readings;
static uint8_t current_paramas_index = 0;
static uint8_t rx_data[30];
static distal_sys_info_t curHmanSysInfo;
// static distal_calib_t curHmanCalibStatus;

static uint64_t expire_nextTime = 0;
static uint8_t prev_fifo_size = 0;

////////////////////////////////////////////////////////////////////////////////
// STM32 PRIVATE FUNCTIONS - VERY IMPORTANT:
////////////////////////////////////////////////////////////////////////////////

#include "stm32_private_funcs"

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
void set_brakes_timed(uint64_t uptime, uint64_t* brakes_next_time);

/////////////////////////////////////////////////////////////////////////////
// MAIN():
/////////////////////////////////////////////////////////////////////////////

int main(void)
{
	/* USER CODE BEGIN 1 */
	uint8_t motor_result = 0;
	uint8_t motor_alert = 0;
	uint8_t prevConnected = 0;
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

	//start UART sys
	/* MINIMAL
	startup_status = uart_sys_init();
	uart_printf("System starting up!\r\n");
	*/

	//start 1ms timer
	startup_status = startBaseTimer();
	if (startup_status) {
		uart_printf("Base Timer init err!\r\n");
	}

	/* MINIMAL
	//start motor driver
	startup_status = motor_qei_sys_start();
	if (startup_status) {
		uart_printf("Motor PWM and QEI init err!\r\n");
	}
	uart_printf("System startup success!\r\n");

	//disable motor
	motor_P_move(0, false, false);
	motor_R_move(0, false, false);

	//start motor algo
	init_motor_algo();
	*/

	//set up ethernet
	Ethernet_Reset(true);
	HAL_Delay(100);
	Ethernet_Reset(false);
	HAL_Delay(1000);
	set_ethernet_w5500_mac(0x00, 0x0a, 0xdc, 0xab, 0xcd, 0xef);
	ethernet_w5500_sys_init();

	/* MINIMAL
	// Hman TCP APP
	distal_tcp_init_app_state(0, VER_H, VER_L, VER_P);
	 */

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	///Cycle LED
	Cycle_LED_Init();

	while (1) {

		//uart rx state check
		uart_rx_data_state();

		//ethernet check
		ethernet_w5500_state();

		//parse input data??
		curHmanSysInfo = distal_tcp_app_state(Read_Haptic_Button(),	motor_alert);

		/* MINIMAL
		//clear motor_alert after sending into tcp app state
		motor_alert = 0;
		*/

		//if system is on or off?
		if (curHmanSysInfo.system_state == ON) {
			//check emergency signal GPIOG GPIO_PIN_14
			if (Read_Haptic_Button()) {
				Left_LED_function (Blue);
				Right_LED_function (Blue);
			} else {
				Left_LED_function (Red);
				Right_LED_function (Red);
			}

			// Brakes status:
			if (getUpTime() >= brakes_nextTime) {
				brakes_nextTime = getUpTime() + 1; //1kHz

				//Code to Engage & Disengage Brake of the Radial Axis
				set_r_brake_status(
						r_brakes(
								get_r_brake_cmd() && Read_Haptic_Button()));

				//Code to Engage & Disengage Brake of the Rotational Axis
				set_p_brake_status(
						p_brakes(
								get_p_brake_cmd() && Read_Haptic_Button()));
			}

			/* MINIMAL
			//udpate safety
			set_safetyOff(curHmanSysInfo.safetyOFF);
			*/

			//check activity state
			switch (curHmanSysInfo.activity_state) {
				case IDLE: {
					/* MINIMAL
					//clear motor algo readings and settings
					clear_distal_mech_readings();
					clear_distal_motors_settings();
					clear_ctrl_params();

					//disable motor
					motor_P_move(0, false, false);
					motor_R_move(0, false, false);

					//restart
					init_motor_algo();

					//restart calibration state
					//reset_calibration_state();
					 */

					break;
				}

				case EXERCISE: {
					if (getUpTime() >= algo_nextTime) {
						algo_nextTime = getUpTime() + DT_STEP_MSEC; //200Hz
						if ((curHmanSysInfo.activity_state == EXERCISE)
								&& (curHmanSysInfo.exercise_state == RUNNING)) {

							/* MINIMAL
							//Retrieve Force Sensor Readings
							force_sensors_read(&hadc3, &X_force_sensor, &Y_force_sensor, &Dummy_X_force_sensor, &Dummy_Y_force_sensor);
							current_sensors_read(&hadc1, &R_current_sensor, &P_current_sensor);

							//update motor driver algo
							motor_result = update_motor_algo(curHmanSysInfo.exercise_mode, qei_count_R_read(),
									qei_count_P_read(), X_force_sensor, Y_force_sensor,
									R_current_sensor, P_current_sensor,
									get_r_brake_cmd(), get_p_brake_cmd(), getUpTime(), 1);

							//update motor alert
							if (motor_result == 0xF0)
								motor_alert = 1;
							else if (motor_result == 0xF1)
								motor_alert = 2;
							else
								motor_alert = 0;

							//reset
							motor_result = 0;

							//update feedback array in Hman TCP
							get_distal_mech_readings(&current_mech_readings); //get latest algo settings
							get_distal_motors_settings(&current_motors_settings); //get latest algo settings

							//check if need to end exercise
							if ((motor_alert == 1) || (motor_alert == 2)) {
								stop_exercise();

								//disable motor
								motor_P_move(0, false, false);
								motor_R_move(0, false, false);
							} else {
								motor_P_move(current_motors_settings.right.dac_in, current_motors_settings.right.motor_direction,
										current_motors_settings.right.en_motor_driver);
								motor_R_move(current_motors_settings.left.dac_in, current_motors_settings.left.motor_direction,
										current_motors_settings.left.en_motor_driver);
							}

							//Distal Force Sensor - Change only when updating TCP Protocol
							//Input Brakes info from TCP System Info
							set_distal_exercise_feedback_info(current_mech_readings.coord.x,
									current_mech_readings.coord.y,
									current_mech_readings.left.qei_count,
									current_mech_readings.right.qei_count,
									current_mech_readings.velocity.x,
									current_mech_readings.velocity.y,
									current_motors_settings.left.volt,
									current_motors_settings.right.volt,
									current_mech_readings.left.currsens_amps,
									current_mech_readings.right.currsens_amps,
									current_mech_readings.Xforce, //X-axis Force Sensor
									current_mech_readings.Yforce, //Y-axis Force Sensor
									current_motors_settings.force.x,
									current_motors_settings.force.y,
									current_motors_settings.force_magnitude);
							*/

						}
						else {
							/* MINIMAL
							// reset
							clear_distal_mech_readings();
							clear_distal_motors_settings();
							clear_ctrl_params();

							//Set the motors to 0 and disable the motor driver
							motor_P_move(0, false, false);
							motor_R_move(0, false, false);
							*/
						}
					}
					break;
				}
			}

			//indicate system was TCP connected
			prevConnected = 1;

		} else {//disconnected
			//set all LED to OFF
			HAL_GPIO_WritePin(L_BLUE_GPIO_Port, L_BLUE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(R_BLUE_GPIO_Port, R_BLUE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(R_RED_GPIO_Port, R_RED_Pin, GPIO_PIN_RESET);

			/* MINIMAL
			//reset
			clear_distal_mech_readings();
			clear_distal_motors_settings();
			clear_ctrl_params();
			reset_force_sensor_calib_data();

			//disable motor
			motor_P_move(0, false, false);
			motor_R_move(0, false, false);
			*/

			r_brakes(false);
			p_brakes(false);

			/* MINIMAL
			//restart
			init_motor_algo();
			*/
		}

		/////////////////////////////////////////////////////////////////////////////////////
		//check if need to dump UART FIFO
		/////////////////////////////////////////////////////////////////////////////////////

		/* MINIMAL
		if (prev_fifo_size != rx_fifo_size()) {
			prev_fifo_size = rx_fifo_size();
			expire_nextTime = getUpTime() + 1000;
		}
		if ((getUpTime() >= expire_nextTime) && (rx_fifo_size() > 0)) {
			rx_fifo_clear();
			prev_fifo_size = 0;
		}
		*/
	}
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

void
set_brakes_timed(uint64_t uptime, uint64_t* brakes_next_time) {
	if (uptime >= *brakes_next_time) {
		*brakes_next_time = uptime + 1; // 1kHz

		/////////////////////////////////////////////////////////////////////////////////////
		// Code to Engage & Disengage Brake of the Radial Axis
		/////////////////////////////////////////////////////////////////////////////////////

		set_l_brake_status(r_brakes(get_l_brake_cmd() && Read_Haptic_Button()));

		/////////////////////////////////////////////////////////////////////////////////////
		// Code to Engage & Disengage Brake of the Rotational Axis
		/////////////////////////////////////////////////////////////////////////////////////

		set_r_brake_status(p_brakes(get_r_brake_cmd() && Read_Haptic_Button()));
	}
}


