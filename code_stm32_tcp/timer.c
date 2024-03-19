/* ========================================
 *
 * Copyright Thesis Pte Ltd, 2017
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "timer.h"

extern TIM_HandleTypeDef htim9;
TIM_HandleTypeDef *distal_htim = &htim9;

volatile uint64_t ui64UpTimeMS;
volatile uint64_t ui64Unix;
/*
1ms ticks interrupt
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == distal_htim->Instance)
	{
		/* Increment the Counter to indicate the keep track of the number of 
     * interrupts received */
    ui64UpTimeMS++;
    
    //RTC will be updated
    if((ui64UpTimeMS%ONE_SEC_IN_MS) == 0)
		{
			ui64Unix++;
		}
	}
}

/*
Return 1ms up time
param[in]: none
param[out]: ui64UpTimeMS
*/
uint64_t getUpTime(void)
{
    return ui64UpTimeMS;
}

/**
* @brief: Set Unix
* @param[in]: uint64 Unix
*/
void set_TIM_unix(uint64_t tmp)
{
	ui64Unix = tmp;
}

/**
* @brief: Get Unix
* @retval: uint64 Unix
*/
uint64_t  get_TIM_unix(void)
{
	return ui64Unix;
}

/*
Start the base timer
param[in]: none
param[out]: none
@retval: 0 = OK. Non-zero = error;
*/
uint8_t startBaseTimer(void)
{
    //start timer
    ui64UpTimeMS = 0;
    if(HAL_TIM_Base_Start_IT(distal_htim) != HAL_OK){
        /* PWM Generation Error */
        Error_Handler();
				return HAL_ERROR;
    }
		
		//set UNIX
		ui64Unix = 0;
		
		return HAL_OK;
}

/* [] END OF FILE */
