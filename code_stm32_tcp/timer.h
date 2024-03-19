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

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef _TIMER_H_
#define _TIMER_H_

#define ONE_SEC_IN_MS     1000

/*
	Start the base timer
	param[in]: none
	param[out]: none
	@retval: 0 = OK. Non-zero = error;
	*/
uint8_t startBaseTimer(void);

/*
Return 1ms up time
param[in]: none
param[out]: ui641msUpTime
*/
uint64_t getUpTime(void);

/**
* @brief: Get Unix
* @retval: uint64 Unix
*/
uint64_t  get_TIM_unix(void);

/**
* @brief: Set Unix
* @param[in]: uint64 Unix
*/
void set_TIM_unix(uint64_t tmp);

#endif

/* [] END OF FILE */
