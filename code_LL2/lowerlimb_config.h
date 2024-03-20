/** ========================================

 Copyright Thesis Pte Ltd, 2018
 All Rights Reserved
 UNPUBLISHED, LICENSED SOFTWARE.

 CONFIDENTIAL AND PROPRIETARY INFORMATION
 WHICH IS THE PROPERTY OF THESIS PTE LTD.

 ========================================
 Version: 1.3
 Date: 13 Sep 2018
 Written by: Kenneth Er
 ========================================

 ======================================== */

#ifndef LL2_LOWERLIMB_CONFIG_H_
#define LL2_LOWERLIMB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define UART_DRIVER_EN			1
#define DEBUG_PRINT				0
#define LOWERLIMB_SAFETY			0
#define MOTOR_DRIVER_OPS 		1
#define W5500_DEBUG_PRINTF		1
#define ONBOARD_ETHERNET		0
#define DHCP_SERV_EN			0 //enable DHCP

//IP Configuration
#define MAC_ADDRESS 	{0x00, 0x08, 0xdc, 0xab, 0xcd, 0xed}
#define IP_ADDRESS		{192, 168, 102, 12}
#define PORT_NUMBER		3002

//math definitions
//#define M_PI					3.141592

//Distal Algorithm Settings
#define LOWERLIMB_MAX_TARGETS		20

//#define R_GEARRATIO				5.75
//#define R_PULLEYDIAMETER		40						//in meters
//#define P_GEARRATIO				26
//#define P_PULLEY1				25
//#define P_PULLEY2				36
#define ENC_RES					2000
#define MAX_QEI_CPR				ENC_RES						//500 counts per revolution

#define R_OFFSET				0.0145	//14.5mm DISTANCE FROM MIDDLE OF THE PULLEY TO MID OF HANDLE SHAFT
#define P_OFFSET				0//-1 * M_PI / 2

#define PULLEYDIAMETER			0.040						//in meters
#define GEARRATIO				7

#define MOTOR_FORCE_COMPENSATE 1
//To ease calculation for the Jacobian Matrix / Constants
#define JACOBIAN				(float)PULLEYDIAMETER / ((float) 2 * (float)2 * GEARRATIO)
//#define R_JACOBIAN				R_PULLEYDIAMETER / (2 * R_GEARRATIO)
//#define P_JACOBIAN				P_PULLEY1 / (P_PULLEY2 * P_GEARRATIO)

//Force Sensors Amplifier Parameters
#define AMPLIFIER_MIN					0.004						//Amplifier minimum current in Ampere
#define AMPLIFIER_NOM					0.012						//Amplifier nominal current in Ampere
#define AMPLIFIER_MAX					0.020						//Amplifier maximum current in Ampere
#define AMPLIFIER_RESISTANCE	150							//Resistance at Amplifier out terminal in Ohms
#define AMPLIFIER_BUFFER_V		0

#define AMPLIFIER_MIN_VOLT		AMPLIFIER_MIN * AMPLIFIER_RESISTANCE	//Amplifier minimum current in Volts
#define AMPLIFIER_NOM_VOLT		AMPLIFIER_NOM * AMPLIFIER_RESISTANCE	//Amplifier nominal current in Volts
#define AMPLIFIER_MAX_VOLT		AMPLIFIER_MAX * AMPLIFIER_RESISTANCE	//Amplifier maximum current in Volts

//Force Sensor Calibration Parameters
#define X_FORCE_MEASURED_POS      200
#define X_VOLT_CALCULATED_POS     AMPLIFIER_MAX_VOLT
#define Y_FORCE_MEASURED_POS      X_FORCE_MEASURED_POS
#define Y_VOLT_CALCULATED_POS     X_VOLT_CALCULATED_POS
#define FORCE_REF                 0
#define VOLT_REF                  AMPLIFIER_NOM_VOLT
#define X_FORCE_MEASURED_NEG      -1.0*X_FORCE_MEASURED_POS
#define X_VOLT_CALCULATED_NEG     AMPLIFIER_MIN_VOLT
#define Y_FORCE_MEASURED_NEG      X_FORCE_MEASURED_NEG
#define Y_VOLT_CALCULATED_NEG     X_VOLT_CALCULATED_NEG

//Force Sensor Voltage Correction - Default Values to Check deterioration
#define X_VOLT_ZERO_OFFSET        0
#define Y_VOLT_ZERO_OFFSET        0

//Force Calculation Parameters
#define X_FORCE_VOLT_GRADIENT       (X_FORCE_MEASURED_POS - FORCE_REF)/(X_VOLT_CALCULATED_POS - VOLT_REF)
#define X_FORCE_VOLT_OFFSET         FORCE_REF - (X_FORCE_VOLT_GRADIENT * VOLT_REF)
#define Y_FORCE_VOLT_GRADIENT       (Y_FORCE_MEASURED_POS - FORCE_REF)/(Y_VOLT_CALCULATED_POS - VOLT_REF)
#define Y_FORCE_VOLT_OFFSET         FORCE_REF - (Y_FORCE_VOLT_GRADIENT * VOLT_REF)

#define FORCE_SENSOR_READ_ZERO 		0 // 2233 // TODO: eliminate at a later date

//Control Parameters Gain
#define FB_GAIN					0
#define FF_GAIN					0
#define COMP_GAIN       0

//Motor Driver Input DAC Configuration Parameters
//#define R_MD_T_CONST			0.175				//Nm/A
//#define R_MD_NOM_CURR			0.628
//
//#define R_MD_MAX_CURR			R_MD_NOM_CURR
//#define R_MD_MIN_CURR			0
//
//#define R_MD_MAX_IN_VOLT			3.3
//#define R_MD_MIN_IN_VOLT			0
//
//#define R_MD_MAX_OUT_VOLT			3.0
//#define R_MD_MIN_OUT_VOLT			0.1
//
//#define P_MD_T_CONST			0.266				//Nm/A
//#define P_MD_NOM_CURR			0.73

#define MD_T_CONST			0.0604				//Nm/A
#define MD_NOM_CURR			7.00

#define MD_MAX_CURR			7.00
#define MD_MIN_CURR			0

#define MD_MAX_IN_VOLT			3.290
#define MD_MIN_IN_VOLT			0.01

#define MD_MAX_OUT_VOLT			3.0
#define MD_MIN_OUT_VOLT			0.1

// #define R_MD_OUT_CURRVOLT_GRADIENT	(R_MD_MAX_OUT_VOLT - R_MD_MIN_OUT_VOLT) / (R_MD_MAX_CURR - R_MD_MIN_CURR)
// #define R_MD_OUT_VOLT_OFFSET		R_MD_MAX_OUT_VOLT - (R_MD_OUT_CURRVOLT_GRADIENT * R_MD_MAX_CURR)
#define MD_OUT_CURRVOLT_GRADIENT	(MD_MAX_OUT_VOLT - MD_MIN_OUT_VOLT) / (MD_MAX_CURR - MD_MIN_CURR)
#define MD_OUT_VOLT_OFFSET			MD_MAX_OUT_VOLT - (MD_OUT_CURRVOLT_GRADIENT * MD_MAX_CURR)

// #define R_MD_IN_CURRVOLT_GRADIENT	(R_MD_NOM_CURR - (-1 * R_MD_NOM_CURR)) / (R_MD_MAX_IN_VOLT - R_MD_MIN_IN_VOLT)
// #define R_MD_IN_VOLT_OFFSET				(-1 * R_MD_NOM_CURR) - (R_MD_MIN_IN_VOLT * R_MD_IN_CURRVOLT_GRADIENT)
#define MD_IN_CURRVOLT_GRADIENT		(MD_NOM_CURR - (-1 * MD_NOM_CURR)) / (MD_MAX_IN_VOLT - MD_MIN_IN_VOLT)
#define MD_IN_VOLT_OFFSET			(-1 * MD_NOM_CURR) - (MD_MIN_IN_VOLT * MD_IN_CURRVOLT_GRADIENT)

// Distal device safety limits
#define LOWERLIMB_MAX_BXX          	300                     	// in Ns/m Max value of BXX for which Distal is stable
#define LOWERLIMB_MAX_BYY          	300                    		// in Ns/m Max value of BYY for which Distal is stable
#define LOWERLIMB_MIN_BXX          	(-1 * LOWERLIMB_MAX_BXX)     	// in Ns/m MIN value of BXX for which Distal is stable
#define LOWERLIMB_MIN_BYY          	(-1 * LOWERLIMB_MAX_BYY)     	// in Ns/m MIN value of BYY for which Distal is stable

// Distal device safety limits
#define LOWERLIMB_SAMPLING_MAX		1.1							//in ms
#define LOWERLIMB_RT_MAX_VEL		1.5							//in m/s. Defines the max real-time x y velocities.

enum {
	INSUF_SAMPLING = 0, RT_VEL_SAFETY = 1
};

#ifdef __cplusplus
}
#endif

#endif // ! MOTOR_ALGO_H
