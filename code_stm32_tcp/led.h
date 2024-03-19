/*
 * led.h
 *
 *  Created on: 11 Jul 2023
 *      Author: ASUS
 */

#ifndef LED_H
#define LED_H

#include "pcb_io.h"

typedef enum {
	Off = 0,
	Red = 1,
	Green = 2,
	Yellow = 3,
	Blue = 4,
	Magenta = 5,
	Cyan = 6,
	White = 7
} LED_Colour;

void Left_LED_function (LED_Colour colour);

void Right_LED_function (LED_Colour colour);

void Cycle_LED_Init (void);

#endif /* ALL_CODES_LED_H_ */
