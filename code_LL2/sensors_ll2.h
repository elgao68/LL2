/////////////////////////////////////////////////////////////////////////////
//
// sensors_ll2.h
//
// Created on: 2024.07.08
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2_SENSORS_LL2_H_
#define CODE_LL2_SENSORS_LL2_H_

#include <_std_c.h>
#include <motor_algo_ll2.h> // HACK: included so that calls to motor_algo_ll2.h functions don't give surprises
#include "stm32f4xx_hal.h"

void force_sensors_zero_calibrate(ADC_HandleTypeDef* hadc3);

#endif /* CODE_LL2_SENSORS_LL2_H_ */
