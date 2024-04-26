/*
 * pcb_io.c
 *
 *  Created on: Jul 6, 2023
 *      Author: Jonathan
 */

#include "pcb_io.h"
#include "stm32f4xx_hal_gpio.h"

#define LEFT_MD_CHANNEL DAC_CHANNEL_2
#define RIGHT_MD_CHANNEL DAC_CHANNEL_1

bool Read_Haptic_Button(void){
	return HAL_GPIO_ReadPin(SIG_EMGC_GPIO_Port, SIG_EMGC_Pin);
}

void Ethernet_ChipSelect (bool input){
	HAL_GPIO_WritePin(ETHERNET_SCSn_GPIO_Port, ETHERNET_SCSn_Pin, input);
}

void Ethernet_Reset (bool input){
	HAL_GPIO_WritePin(ETHERNET_RSTn_GPIO_Port, ETHERNET_RSTn_Pin, !input);
}

void Left_Red_LED (bool input){
	HAL_GPIO_WritePin(L_RED_GPIO_Port, L_RED_Pin, input);
}

void Left_Blue_LED (bool input){
	HAL_GPIO_WritePin(L_BLUE_GPIO_Port, L_BLUE_Pin, input);
}

void Left_Green_LED (bool input){
	HAL_GPIO_WritePin(L_GREEN_GPIO_Port, L_GREEN_Pin, input);
}

void Right_Red_LED (bool input){
	HAL_GPIO_WritePin(R_RED_GPIO_Port, R_RED_Pin, input);
}

void Right_Blue_LED (bool input){
	HAL_GPIO_WritePin(R_BLUE_GPIO_Port, R_BLUE_Pin, input);
}

void Right_Green_LED (bool input){
	HAL_GPIO_WritePin(R_GREEN_GPIO_Port, R_GREEN_Pin, input);
}

void Left_Motor_Brakes_Powersave (bool input){
	HAL_GPIO_WritePin(L_BRAKE_16V_GPIO_Port, L_BRAKE_16V_Pin, input);
}

void Left_Motor_Brakes_Disengage (bool input){
	HAL_GPIO_WritePin(L_BRAKE_24V_GPIO_Port, L_BRAKE_24V_Pin, input);
}

void Right_Motor_Brakes_Powersave (bool input){
	HAL_GPIO_WritePin(R_BRAKE_16V_GPIO_Port, R_BRAKE_16V_Pin, input);
}

void Right_Motor_Brakes_Disengage (bool input){
	HAL_GPIO_WritePin(R_BRAKE_24V_GPIO_Port, R_BRAKE_24V_Pin, input);
}

//Left Motor Driver - Radial Motor
void Left_Motor_SetValue (uint32_t input){
	HAL_DAC_SetValue(MD_ANALOGIN_HANDLE, LEFT_MD_CHANNEL, DAC_ALIGN_12B_R, input);
}

void Left_Motor_Enable (bool input){
	HAL_GPIO_WritePin(L_MD_DI2_GPIO_Port, L_MD_DI2_Pin, input);
}

void Left_Motor_Direction (bool input){
	HAL_GPIO_WritePin(L_MD_DIO3_GPIO_Port, L_MD_DIO3_Pin, input);
}

//Right Motor Driver - Angular Motor
void Right_Motor_SetValue (uint32_t input){
	HAL_DAC_SetValue(MD_ANALOGIN_HANDLE, RIGHT_MD_CHANNEL, DAC_ALIGN_12B_R, input);
}

void Right_Motor_Enable (bool input){
	HAL_GPIO_WritePin(R_MD_DI2_GPIO_Port, R_MD_DI2_Pin, input);
}

void Right_Motor_Direction (bool input){
	HAL_GPIO_WritePin(R_MD_DIO3_GPIO_Port, R_MD_DIO3_Pin, input);
}





