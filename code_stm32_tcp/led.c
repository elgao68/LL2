/*
 * led.c
 *
 *  Created on: 11 Jul 2023
 *      Author: ASUS
 */

#include "led.h"


void Left_LED_function (LED_Colour colour){
	Left_Red_LED (colour & 0x01);
	Left_Green_LED (colour & 0x02);
	Left_Blue_LED (colour & 0x04);
}

void Right_LED_function (LED_Colour colour){
	Right_Red_LED (colour & 0x01);
	Right_Green_LED (colour & 0x02);
	Right_Blue_LED (colour & 0x04);
}

void Cycle_LED_Init (void){
	Left_Red_LED (true);
	HAL_Delay(250);
	Left_Blue_LED (true);
	HAL_Delay(250);
	Left_Green_LED (true);
	HAL_Delay(250);
	Right_Red_LED (true);
	HAL_Delay(250);
	Right_Blue_LED (true);
	HAL_Delay(250);
	Right_Green_LED (true);
	HAL_Delay(250);
	Left_Red_LED (false);
	HAL_Delay(250);
	Left_Blue_LED (false);
	HAL_Delay(250);
	Left_Green_LED (false);
	HAL_Delay(250);
	Right_Red_LED (false);
	HAL_Delay(250);
	Right_Blue_LED (false);
	HAL_Delay(250);
	Right_Green_LED (false);
	HAL_Delay(250);
}

void Transition_LED (bool start){
	static int count = 0;
	static LED_Colour colour = Red;

	if (start){
		if (count == 33){
			count = 0;

			switch(colour){
				case Red:
					colour = Yellow;
					break;
				case Yellow:
					colour = Green;
					break;
				case Green:
					colour = Cyan;
					break;
				case Cyan:
					colour = Blue;
					break;
				case Blue:
					colour = Magenta;
					break;
				case Magenta:
					colour = Red;
					break;
				default:
					break;
			}
		}

		count++;

	} else{

	}
}
