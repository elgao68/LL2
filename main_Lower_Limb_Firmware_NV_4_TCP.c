/////////////////////////////////////////////////////////////////////////////
//
// main.c
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
// #define INTERVAL_5MS		5

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
// Private function prototypes:
////////////////////////////////////////////////////////////////////////////////

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);


void motor_test_1(void);
void adc_test_1(void);
int ADC_test_oneshot(float *cs1, float *cs2, float *cs3, float *cs4);


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
/////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS - DEFINITIONS - GAO
/////////////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS:
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	*/
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = ENABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ENABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = ENABLE;
	hadc3.Init.NbrOfDiscConversion = 1;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 4;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/* DAC init function */
static void MX_DAC_Init(void)
{
	DAC_ChannelConfTypeDef sConfig;

	/**DAC Initialization
	*/
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**DAC channel OUT1 config
	*/
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**DAC channel OUT2 config
	*/
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 5;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 5;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 5;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 5;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 179;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 1000;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
	huart3.Init.BaudRate = 230400;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, R_MD_DIO3_Pin|R_MD_DI2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, R_RED_Pin|R_BLUE_Pin|R_GREEN_Pin|L_MD_DIO3_Pin
						  |L_MD_DI2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, L_BRAKE_16V_Pin|L_BRAKE_24V_Pin|R_BRAKE_16V_Pin|R_BRAKE_24V_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, L_GREEN_Pin|L_BLUE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, L_RED_Pin|ETHERNET_SCSn_Pin|ETHERNET_RSTn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PE2 PE3 PE4 PE5
						   PE6 PE7 PE8 PE9
						   PE14 PE15 PE0 PE1 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
						  |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
						  |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PC13 PC14 PC15 PC5
						   PC6 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5
						  |GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PF0 PF1 PF2 PF6
						   PF7 PF10 PF11 PF12
						   PF13 PF14 PF15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6
						  |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
						  |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : R_MD_DIO3_Pin R_MD_DI2_Pin */
	GPIO_InitStruct.Pin = R_MD_DIO3_Pin|R_MD_DI2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : PH0 PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : R_RED_Pin R_BLUE_Pin R_GREEN_Pin L_MD_DIO3_Pin
						   L_MD_DI2_Pin */
	GPIO_InitStruct.Pin = R_RED_Pin|R_BLUE_Pin|R_GREEN_Pin|L_MD_DIO3_Pin
						  |L_MD_DI2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : R_QEI_INDEX_Pin */
	GPIO_InitStruct.Pin = R_QEI_INDEX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(R_QEI_INDEX_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA3 PA6 PA7 PA8
						   PA9 PA10 PA11 PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
						  |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1 PB2 PB12 PB13
						   PB5 PB6 PB7 PB8
						   PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
						  |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
						  |GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PG0 PG1 PG2 PG3
						   PG4 PG5 PG6 PG7
						   PG8 PG9 PG10 PG11
						   PG12 PG13 PG15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
						  |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
						  |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
						  |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : L_BRAKE_16V_Pin L_BRAKE_24V_Pin R_BRAKE_16V_Pin R_BRAKE_24V_Pin */
	GPIO_InitStruct.Pin = L_BRAKE_16V_Pin|L_BRAKE_24V_Pin|R_BRAKE_16V_Pin|R_BRAKE_24V_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : L_GREEN_Pin L_BLUE_Pin */
	GPIO_InitStruct.Pin = L_GREEN_Pin|L_BLUE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : L_RED_Pin ETHERNET_RSTn_Pin */
	GPIO_InitStruct.Pin = L_RED_Pin|ETHERNET_RSTn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PD9 PD10 PD14 PD15
						   PD3 PD4 PD5 PD6
						   PD7 */
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15
						  |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
						  |GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : L_QEI_INDEX_Pin ETHERNET_INTn_Pin */
	GPIO_InitStruct.Pin = L_QEI_INDEX_Pin|ETHERNET_INTn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : ETHERNET_SCSn_Pin */
	GPIO_InitStruct.Pin = ETHERNET_SCSn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ETHERNET_SCSn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SIG_EMGC_Pin */
	GPIO_InitStruct.Pin = SIG_EMGC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SIG_EMGC_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/*
 //ADC test 1

 Collect 1000 samples at 1ms and double check output using ADC injected channels
 */
void adc_test_1(void) {
	uint16_t chn1[10];
	uint16_t chn2[10];
	uint16_t chn3[10];
	uint16_t chn4[10];
	int32_t chn1mV[10];
	int32_t chn2mV[10];
	int32_t chn3mV[10];
	int32_t chn4mV[10];
	uint16_t n_cnt = 0;
	HAL_StatusTypeDef test_status_adc1 = 0;
	uint64_t sampleNextTime = 0;
	uint64_t avgDeltaTime = 0;
	uint8_t rx_byte;

	//clear
	memset(chn1, 0, sizeof(chn1));
	memset(chn2, 0, sizeof(chn2));
	memset(chn3, 0, sizeof(chn3));
	memset(chn4, 0, sizeof(chn4));

//	uart_printf("ADC test 1 - collect 10 samples @ 1kHz per channel\r\n");
	uart_printf("ADC test 1 for 4 channels. Follow instruction\r\n");
	uart_printf("Channel 1 = L Motor driver CS\r\n");
	uart_printf("Channel 2 = L Hall Effect Sensor CS\r\n");
	uart_printf("Channel 3 = R Motor driver CS\r\n");
	uart_printf("Channel 4 = R Hall Effect Sensor CS\r\n");

	//instruction
	uart_printf("Short Channel 1 to GND. Enter 'A' or 'a' when ready.\r\n");
	while (1) {
		//check for data
		uart_rx_data_state();

		//echo
		if ((rx_fifo_size() > 0) && (is_uart_busy() == 0)) {
			rx_fifo_dequeue(&rx_byte);
			if ((rx_byte == 'A') || (rx_byte == 'a')) {
				break;
			}
		}
	}

	//next time
	sampleNextTime = getUpTime() + 1; //1ms

	for (n_cnt = 0; n_cnt < 10; n_cnt++) {
//		if(getUpTime() > sampleNextTime)
//		{
		//delta time
		avgDeltaTime += (getUpTime() - sampleNextTime);

		//next time
		sampleNextTime = getUpTime() + 1; //1ms

		//start ADC
		test_status_adc1 = HAL_ADCEx_InjectedStart(&hadc1);
		if (test_status_adc1 != HAL_OK) {
			uart_printf("ADC start failed @ %d cnt. Error = %d\r\n", n_cnt,
					test_status_adc1);
			while (1) {
			};
		}

		test_status_adc1 = HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);
		if (test_status_adc1 != HAL_OK) {
			uart_printf("ADC conversion failed @ %d cnt. Error = %d\r\n", n_cnt,
					test_status_adc1);
			while (1) {
			};
		}

		//store results
		chn1[n_cnt] = (uint16_t) HAL_ADCEx_InjectedGetValue(&hadc1,
		ADC_INJECTED_RANK_1);
		chn2[n_cnt] = (uint16_t) HAL_ADCEx_InjectedGetValue(&hadc1,
		ADC_INJECTED_RANK_2);
		chn3[n_cnt] = (uint16_t) HAL_ADCEx_InjectedGetValue(&hadc1,
		ADC_INJECTED_RANK_3);
		chn4[n_cnt] = (uint16_t) HAL_ADCEx_InjectedGetValue(&hadc1,
		ADC_INJECTED_RANK_4);

		//start ADC
		test_status_adc1 = HAL_ADCEx_InjectedStop(&hadc1);
		if (test_status_adc1 != HAL_OK) {
			uart_printf("ADC stop failed @ %d cnt. Error = %d\r\n", n_cnt,
					test_status_adc1);
			while (1) {
			};
		}
//		}

		if (n_cnt == 1) {
			uart_printf("Short Channel 1 to +3.3V. Enter 'A' or 'a' when ready.\r\n");
			while (1) {
				//check for data
				uart_rx_data_state();

				//echo
				if ((rx_fifo_size() > 0) && (is_uart_busy() == 0)) {
					rx_fifo_dequeue(&rx_byte);
					if ((rx_byte == 'A') || (rx_byte == 'a')) {
						break;
					}
				}
			}
		} else if (n_cnt == 3) {
			uart_printf("Short Channel 3 to GND. Enter 'A' or 'a' when ready.\r\n");
			while (1) {
				//check for data
				uart_rx_data_state();

				//echo
				if ((rx_fifo_size() > 0) && (is_uart_busy() == 0)) {
					rx_fifo_dequeue(&rx_byte);
					if ((rx_byte == 'A') || (rx_byte == 'a')) {
						break;
					}
				}
			}
		} else if (n_cnt == 5) {
			uart_printf("Short Channel 3 to +3.3V. Enter 'A' or 'a' when ready.\r\n");
			while (1) {
				//check for data
				uart_rx_data_state();

				//echo
				if ((rx_fifo_size() > 0) && (is_uart_busy() == 0)) {
					rx_fifo_dequeue(&rx_byte);
					if ((rx_byte == 'A') || (rx_byte == 'a')) {
						break;
					}
				}
			}
		}
	}

	//convert all result to mV
	for (n_cnt = 0; n_cnt < 10; n_cnt++) {
		chn1mV[n_cnt] = (int32_t) chn1[n_cnt] * 3300 / 4096;
		chn2mV[n_cnt] = (int32_t) chn2[n_cnt] * 3300 / 4096;
		chn3mV[n_cnt] = (int32_t) chn3[n_cnt] * 3300 / 4096;
		chn4mV[n_cnt] = (int32_t) chn4[n_cnt] * 3300 / 4096;
	}

	//print result
	uart_printf("Channel 1[0 - 1] should be ~0mV. Result = %d\r\n",
			(chn1mV[0] + chn1mV[1]) / 2);
	uart_printf("Channel 1[2 - 3] should be ~3300mV. Result = %d\r\n",
			(chn1mV[2] + chn1mV[3]) / 2);
	uart_printf("Channel 2 should be ~1250mV � 140mV. Result = %d\r\n",
			(chn2mV[7] + chn2mV[8]) / 2);
	uart_printf("Channel 3[4 - 5] should be ~0mV. Result = %d\r\n",
			(chn3mV[4] + chn3mV[5]) / 2);
	uart_printf("Channel 3[6 - 7] should be ~3300mV. Result = %d\r\n",
			(chn3mV[6] + chn3mV[7]) / 2);
	uart_printf("Channel 4 should be ~1250mV � 140mV. Result = %d\r\n",
			(chn4mV[7] + chn4mV[8]) / 2);

	//get avg delta time
	avgDeltaTime = avgDeltaTime / n_cnt;

	uart_printf("ADC completed with %d ms delta.\r\n", (uint32_t) avgDeltaTime);

	avgDeltaTime = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */

void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
