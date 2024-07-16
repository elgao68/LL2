/////////////////////////////////////////////////////////////////////////////
//
// qei_motor_drivers_LL2.h
//
// Created on: 2024.04.24
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef QEI_MOTOR_DRIVERS_LL2_H
#define QEI_MOTOR_DRIVERS_LL2_H

// Brakes_state:
enum {
	B_ENGAGE = 0,
	B_POWERSAVE = 1,
	B_DISENGAGE = 2
};

////////////////////////////////////////////////////////////////
// MOTOR FUNCTIONS - LL2:
////////////////////////////////////////////////////////////////

uint8_t motor_qei_sys_start(void);

void qei_count_L_reset(void);
void qei_count_R_reset(void);

int32_t qei_count_L_read(void);
int32_t qei_count_R_read(void);

void motor_L_move(uint32_t dac_in, bool dir, bool en);
void motor_R_move(uint32_t dac_in, bool dir, bool en);

void l_brakes_enable();
void r_brakes_enable();

void l_brakes_powersave();
void r_brakes_powersave();

void l_brakes_disable();
void r_brakes_disable();

uint8_t l_brakes(bool disengage);
uint8_t r_brakes(bool disengage);

////////////////////////////////////////////////////////////////
// SENSOR FUNCTIONS:
////////////////////////////////////////////////////////////////

bool force_sensors_read(ADC_HandleTypeDef* hadc, uint32_t* left_x, uint32_t * left_y, uint32_t* right_x, uint32_t * right_y);
bool current_sensors_read(ADC_HandleTypeDef* hadc, uint32_t* left, uint32_t * right);

#endif
