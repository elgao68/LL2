/* ========================================
 
  Copyright Thesis Pte Ltd, 2018
  All Rights Reserved
  UNPUBLISHED, LICENSED SOFTWARE.
 
  CONFIDENTIAL AND PROPRIETARY INFORMATION
  WHICH IS THE PROPERTY OF THESIS PTE LTD.
 
  ========================================
  Version: 1.2
	Date: 31st July 2018
  Written by: Kenneth Er
  ========================================
	PWM, Motor GPIO pins control and QEI
	for STM32F429
	
	======================================== */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "qei_motor_drivers.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "pcb_io.h"

//Definitions
#define CHANNEL_R	TIM_CHANNEL_1	//set to the correct TIM module channel
#define CHANNEL_P	TIM_CHANNEL_2

//Global variables
#ifdef STM32F439xx
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
#elif STM32F429xx //define for the nucleo144 evm kit
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
#endif
extern DAC_HandleTypeDef hdac;

//Private variables
#ifdef STM32F439xx
static TIM_HandleTypeDef *QEI_P = &htim2;
static TIM_HandleTypeDef *QEI_R = &htim4;
#elif STM32F429xx //define for the nucleo144 evm kit
static TIM_HandleTypeDef *QEI_P = &htim1;
static TIM_HandleTypeDef *QEI_R = &htim2;
#endif

static uint16_t qei_R_prev;
static uint16_t qei_P_prev;
static int8_t qei_R_overflow;
static int8_t qei_P_overflow;
static uint64_t R_brakes_powersavetimer = 0;
static uint64_t P_brakes_powersavetimer = 0;

/* 	@brief: Start the motor drivers pwm and QEI
		@retval: 0 = OK. Non-zero = error;
*/
uint8_t motor_qei_sys_start(void)
{
	//set to 0
    motor_R_move(0,false,false);
    motor_L_move(0,false,false);
	
	//start QEI
	if(HAL_TIM_Encoder_Start(QEI_P, TIM_CHANNEL_ALL) != HAL_OK)
	{
        Error_Handler();
		return HAL_ERROR;
    }
			
	if(HAL_TIM_Encoder_Start(QEI_R, TIM_CHANNEL_ALL) != HAL_OK)
	{
        Error_Handler();
		return HAL_ERROR;
    }
	
	//reset QEI
	qei_count_R_reset();
	qei_count_P_reset();

	return HAL_OK;
}
/* 	@brief: Reset Radial QEI count
*/
void qei_count_R_reset(void)
{
	QEI_R->Instance->CNT = 0;
	qei_R_prev = 0;
	qei_R_overflow = 0;
}
/* 	@brief: Read Radial QEI count
*/
int32_t qei_count_L_read(void)
{
	uint16_t tmp;
	int32_t ret_val;
//	__HAL_LOCK(QEI_L);
	tmp = (uint16_t)__HAL_TIM_GET_COUNTER(QEI_R);
	
	//Added
		if ((tmp - qei_R_prev) > 32768)
		{
			qei_R_overflow--;
		}
		else if ((tmp - qei_R_prev) < -32768)
		{
			qei_R_overflow++;
		}
		
	ret_val = tmp + (qei_R_overflow * 65536);
	
//	ret_val = ret_val * (-1); //invert
//	__HAL_UNLOCK(QEI_L);
	
	//Added
	qei_R_prev = tmp;
	
	return ret_val;
}

/* 	@brief: Reset Phi QEI count
*/
void qei_count_P_reset(void)
{
	QEI_P->Instance->CNT = 0;
	qei_P_prev = 0;
	qei_P_overflow = 0;
}

/* 	@brief: Read Phi QEI count
*/
int32_t qei_count_R_read(void)
{
	uint16_t tmp;
	int32_t ret_val;
//	__HAL_LOCK(QEI_L);
	tmp = (uint16_t)__HAL_TIM_GET_COUNTER(QEI_P);
	
	//Added
		if ((tmp - qei_P_prev) > 32768)
		{
			qei_P_overflow--;
		}
		else if ((tmp - qei_P_prev) < -32768)
		{
			qei_P_overflow++;
		}
		
	ret_val = tmp + (qei_P_overflow * 65536);
	
//	ret_val = ret_val * (-1); //invert
//	__HAL_UNLOCK(QEI_L);
	
	//Added
	qei_P_prev = tmp;
	
	return ret_val;
}

void Radial_Motor_Brakes_Powersave(bool input){
	Left_Motor_Brakes_Powersave(input);
}

void Radial_Motor_Brakes_Disengage(bool input){
	Left_Motor_Brakes_Disengage(input);
}

void Angular_Motor_Brakes_Powersave(bool input){
	Right_Motor_Brakes_Powersave(input);
}

void Angular_Motor_Brakes_Disengage(bool input){
	Right_Motor_Brakes_Disengage(input);
}

void r_brakes_enable()
{
	Radial_Motor_Brakes_Powersave(true);
	Radial_Motor_Brakes_Disengage(true);
}

void p_brakes_enable()
{
	Angular_Motor_Brakes_Powersave(true);
	Angular_Motor_Brakes_Disengage(true);
}

void r_brakes_powersave()
{
	Radial_Motor_Brakes_Powersave(true);
	Radial_Motor_Brakes_Disengage(false);
}

void p_brakes_powersave()
{
	Angular_Motor_Brakes_Powersave(true);
	Angular_Motor_Brakes_Disengage(false);
}

void r_brakes_disable()
{
	Radial_Motor_Brakes_Powersave(false);
	Radial_Motor_Brakes_Disengage(false);
}

void p_brakes_disable()
{
	Angular_Motor_Brakes_Powersave(false);
	Angular_Motor_Brakes_Disengage(false);
}

uint8_t r_brakes(bool disengage)
{
	if (disengage)
	{
		if (R_brakes_powersavetimer < 500)
		{
			r_brakes_enable();
			R_brakes_powersavetimer++;
			return B_DISENGAGE;
		}
		else
		{
			r_brakes_powersave();
			return B_POWERSAVE;
		}
	}
	else
	{
		r_brakes_disable();
		R_brakes_powersavetimer = 0;
		return B_ENGAGE;
	}
}

uint8_t p_brakes(bool disengage)
{
	if (disengage)
	{
		if (P_brakes_powersavetimer < 500)
		{
			p_brakes_enable();
			P_brakes_powersavetimer++;
			return B_DISENGAGE;
		}
		else
		{
			p_brakes_powersave();
			return B_POWERSAVE;
		}
	}
	else
	{
		p_brakes_disable();
		P_brakes_powersavetimer = 0;
		return B_ENGAGE;
	}
}


bool force_sensors_read(ADC_HandleTypeDef* hadc, uint32_t* left_x, uint32_t * left_y, uint32_t* right_x, uint32_t * right_y)
{
	for (int i = 0; i < 4; i++){
		if (HAL_ADC_Start(hadc) != HAL_OK || HAL_ADC_PollForConversion(hadc, 100) != HAL_OK)
		{
			*left_x = -1;
			*left_y = -1;
			*right_x = -1;
			*right_y = -1;
			return false;
		}

		switch(i){
			case 0:
				*left_x = HAL_ADC_GetValue(hadc);
				break;
			case 1:
				*left_y = HAL_ADC_GetValue(hadc);
				break;
			case 2:
				*right_x = HAL_ADC_GetValue(hadc);
				break;
			case 3:
				*right_y = HAL_ADC_GetValue(hadc);
				break;
			default:
				*left_x = -1;
				*left_y = -1;
				*right_x = -1;
				*right_y = -1;
				return false;
		}
	}

	return true;
}

bool current_sensors_read(ADC_HandleTypeDef* hadc, uint32_t* r, uint32_t * p)
{
	//Get force sensor p value and assign to structure
	if (HAL_ADC_Start(hadc) != HAL_OK || HAL_ADC_PollForConversion(hadc, 100) != HAL_OK)
	{
		*r = -1;
		*p = -1;
		return false;
	}
	*r = HAL_ADC_GetValue(hadc);

	//Get force sensor r value and assign to structure
	if (HAL_ADC_Start(hadc) != HAL_OK || HAL_ADC_PollForConversion(hadc, 100) != HAL_OK)
	{
		*r = -1;
		*p = -1;
		return false;
	}
	*p = HAL_ADC_GetValue(hadc);

	return true;
}

void motor_R_move(uint32_t dac_in, bool dir, bool en)
{
	Right_Motor_SetValue (dac_in);
	Right_Motor_Enable (en);
	Right_Motor_Direction (dir);
}

void motor_L_move(uint32_t dac_in, bool dir, bool en)
{
	Left_Motor_SetValue (dac_in);
	Left_Motor_Enable (en);
	Left_Motor_Direction (dir);
}
