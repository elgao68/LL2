/* ========================================
 
  Copyright Thesis Pte Ltd, 2018
  All Rights Reserved
  UNPUBLISHED, LICENSED SOFTWARE.
 
  CONFIDENTIAL AND PROPRIETARY INFORMATION
  WHICH IS THE PROPERTY OF THESIS PTE LTD.
 
  ========================================
  Version: 1.1
	Date: 15th May 2018
  Written by: Kenneth Er
  ========================================
	PWM, Motor GPIO pins control and QEI
	for STM32F429
	
	======================================== */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef QEI_MOTOR_DRIVERS_H
#define QEI_MOTOR_DRIVERS_H

enum
{
		B_ENGAGE = 0,
		B_POWERSAVE = 1,
		B_DISENGAGE = 2
}; //brakes_state

/* 	@brief: Start the motor drivers pwm and QEI
		@retval: 0 = OK. Non-zero = error;
*/
uint8_t motor_qei_sys_start(void);


/* 	@brief: Reset Radial QEI count
*/
void qei_count_R_reset(void);
/* 	@brief: Read Radial QEI count
*/
int32_t qei_count_R_read(void);


/* 	@brief: Reset Phi QEI count
*/
void qei_count_P_reset(void);
/* 	@brief: Read Phi QEI count
*/
int32_t qei_count_P_read(void);

void r_brakes_enable();
void p_brakes_enable();
void r_brakes_powersave();
void p_brakes_powersave();
void r_brakes_disable();
void p_brakes_disable();

uint8_t r_brakes (bool disengage);
uint8_t p_brakes (bool disengage);

bool force_sensors_read(ADC_HandleTypeDef* hadc, uint32_t* left_x, uint32_t * left_y, uint32_t* right_x, uint32_t * right_y);
bool current_sensors_read(ADC_HandleTypeDef* hadc, uint32_t* r, uint32_t * p);

void motor_P_move(uint32_t dac_in, bool dir, bool en);
void motor_R_move(uint32_t dac_in, bool dir, bool en);

#endif

/* [] END OF FILE */
