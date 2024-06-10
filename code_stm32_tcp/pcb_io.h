#ifndef PCB_IO_H
#define PCB_IO_H

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include "main.h"

//#define MD_ANALOGOUT_HANDLE	&hadc1
//#define FS_ANALOGOUT_HANDLE	&hadc3
#define MD_ANALOGIN_HANDLE	&hdac
//#define ETHERNET_SPI_HANDLE	&hspi3
//#define QEI_LIGHT_HANDLE	&htim2
//#define QEI_LEFT_HANDLE	&htim4
//#define TIMER_HANDLE	&htim9
//#define UART_HANDLE	&huart3
//
//extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc3;
extern DAC_HandleTypeDef hdac;
//extern SPI_HandleTypeDef hspi3;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim4;
//extern TIM_HandleTypeDef htim9;
//extern UART_HandleTypeDef huart3;

typedef struct{
	uint32_t in8;
	uint32_t in14;
} ADC1_;

typedef struct{
	uint32_t in9;
	uint32_t in10;
	uint32_t in14;
	uint32_t in15;
} ADC3_;

//Input
bool Read_Haptic_Button(void);

//Left Button LED
void Left_Red_LED (bool input);
void Left_Blue_LED (bool input);
void Left_Green_LED (bool input);

//Right Button LED
void Right_Red_LED (bool input);
void Right_Blue_LED (bool input);
void Right_Green_LED (bool input);

//Radial Motor Brakes
void Left_Motor_Brakes_Powersave (bool input);
void Left_Motor_Brakes_Disengage (bool input);

//Angular Motor Brakes
void Right_Motor_Brakes_Powersave (bool input);
void Right_Motor_Brakes_Disengage (bool input);

//Left Motor Driver - Radial Motor
void Left_Motor_SetValue (uint32_t input);
void Left_Motor_Enable (bool input);
void Left_Motor_Direction (bool input);

//Right Motor Driver - Angular Motor
void Right_Motor_SetValue (uint32_t input);
void Right_Motor_Enable (bool input);
void Right_Motor_Direction (bool input);

//Ethernet IO
void Ethernet_ChipSelect (bool input);
void EtherneRESET_TRAJ_TIMER (bool input);

#endif

