/* ========================================
*
* Copyright Thesis Pte Ltd, 2018
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* CONFIDENTIAL AND PROPRIETARY INFORMATION
* WHICH IS THE PROPERTY OF Thesis Pte Ltd.
*
* Version: 1.11
* Updated: 9 Oct 2018
*
* ========================================
* Updated to match "H-man motor control algorithm v1.0_MA_v2"
* Not implemented: Real time velocity filter
* ========================================
*/

#ifndef  MOTOR_ALGO_H
#define  MOTOR_ALGO_H

#include <_coord_system.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <admitt_model_params.h>
#include <dynamic_systems_nml.h>
#include <lowerlimb_config.h>
#include <traj_ctrl_params_nml.h>

#ifdef __cplusplus
	extern "C" {
#endif

/////////////////////////////////////////////////////////////////////////////////////
// CONTROL TIME STEP - GAO
/////////////////////////////////////////////////////////////////////////////////////

#define DT_STEP_MSEC   5
#define MSEC_PER_SEC   1000

/////////////////////////////////////////////////////////////////////////////////////
// TYPE DEFINITIONS:
/////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    float fbgain;
    float ffgain;
    float compgain;
} ctrl_params_t;

typedef struct {
    float x;
    float y;
} lowerlimb_rp_t;

typedef struct {
    float torque;				//set torque in N.m
    float current;				//calc current flow in Ampere
    float volt;					//calc voltage in V

    uint32_t dac_in;
    bool en_motor_driver;		//True to enable; false to disable
	bool motor_direction;		//False for CCW; True for CW
} lowerlimb_motor_settings_t;	//motor specific struct

typedef struct {
    lowerlimb_motor_settings_t left;
    lowerlimb_motor_settings_t right;
    float force_end[N_COORD_2D];       //calculated force
} lowerlimb_motors_settings_t;

typedef struct {
	int32_t qei_count;				//QEI count read
	float angular_pos_rad;			//angular postion in rad
	float prev_angular_pos_rad;		//prev angular postion in rad
	float angular_vel_rad;			//angular postion in rad/s

	float currsens_volt;
	float currsens_amps;
} lowerlimb_motor_readings_t;

typedef struct {
	lowerlimb_rp_t coord;			//current coordinates in m
	lowerlimb_rp_t prev_coord;		//previous coordinates in m
	lowerlimb_rp_t remaining;		//remaining x and y dist between current and final in m
	lowerlimb_rp_t velocity;		//instant velocity in m/s

    lowerlimb_motor_readings_t left;
    lowerlimb_motor_readings_t right;
		
    uint64_t prev_update_time;	//in ms
    uint64_t update_time;		//in ms

    float Xforce_volt;
    float Xforce_volt_corrected;
    float Xforce;

    float Yforce_volt;
    float Yforce_volt_corrected;
    float Yforce;

    uint8_t init;					//true = init exercise
} lowerlimb_mech_readings_t;

typedef struct {
  float X_axis;
  float Y_axis;
} force_sensor_list_t;

// Reference kinematics:
typedef struct {
	double p_ref[N_COORD_2D];
	double dt_p_ref[N_COORD_2D];
	double phi_ref;
	double dt_phi_ref;
} lowerlimb_ref_kinematics_t;

/////////////////////////////////////////////////////////////////////////////////////
// ODE PARAMETER INDICES:
/////////////////////////////////////////////////////////////////////////////////////

#define IDX_PAR_NML_A_CON 	0
#define IDX_PAR_NML_B_CON 	1

/////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS, MOTION ALGORITHMS - GAO
/////////////////////////////////////////////////////////////////////////////////////

void traj_reference_step_active(
		double p_ref[],	double dt_p_ref[], double* phi_ref, double* dt_phi_ref, double u_t_ref[], double dt_k,
		double F_end_m[],
		traj_ctrl_params_t traj_ctrl_params, admitt_model_params_t admitt_model_params, int USE_ADMITT_MODEL_CONSTR);

/////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS, DYNAMIC SYSTEMS - GAO
/////////////////////////////////////////////////////////////////////////////////////

nml_mat* ode_admitt_model_nml(nml_mat* z, ode_param_struct ode_params);

void admitt_model_matrices_nml(nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, double m_val[], double b_val[], double k_val[], size_t N_q);

/////////////////////////////////////////////////////////////////////////////////////
// LOWER-LIMB UTILITY FUNCTIONS - GAO
/////////////////////////////////////////////////////////////////////////////////////

uint8_t set_LL_mech_readings(lowerlimb_mech_readings_t* mech, uint64_t current_upTime,
		int32_t L_count, int32_t R_count,
		uint32_t L_currsens, uint32_t R_currsens,
		uint32_t force_end_in_x, uint32_t force_end_in_y,
		uint8_t isCalibration);

void set_LL_motor_settings(lowerlimb_motors_settings_t* motors, float force_end[]);

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS, LOW LEVEL - GAO
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
// Safety functions:
/////////////////////////////////////////////////////////////////////////////////////

void set_safetyOff(uint8_t en);
uint8_t get_safetyOff(void);

/////////////////////////////////////////////////////////////////////////////////////
// Sensor readings:
/////////////////////////////////////////////////////////////////////////////////////

void clear_lowerlimb_mech_readings(lowerlimb_mech_readings_t* LL_mech_readings);

/////////////////////////////////////////////////////////////////////////////////////
// Motor settings:
/////////////////////////////////////////////////////////////////////////////////////

void clear_lowerlimb_motors_settings(lowerlimb_motors_settings_t* LL_motors_settings);
uint32_t get_motor_L_dac(lowerlimb_motors_settings_t* LL_motors_settings);
uint32_t get_motor_R_dac(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_L_direction(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_R_direction(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_L_enable(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_R_enable(lowerlimb_motors_settings_t* LL_motors_settings);

/////////////////////////////////////////////////////////////////////////////////////
// Exercise functions:
/////////////////////////////////////////////////////////////////////////////////////

void init_motor_algo(lowerlimb_mech_readings_t* LL_mech_readings, lowerlimb_motors_settings_t* LL_motors_settings);
uint8_t lowerlimb_motor_safety_limiter(uint8_t type, uint8_t targ_index);

/////////////////////////////////////////////////////////////////////////////////////
// Control parameters:
/////////////////////////////////////////////////////////////////////////////////////

void configure_ctrl_params(float fbgain, float ffgain, float compgain);
void clear_ctrl_params();

/////////////////////////////////////////////////////////////////////////////////////
// Offset forces:
/////////////////////////////////////////////////////////////////////////////////////

void set_force_sensor_zero_offset (float X_axis, float Y_axis);
// void reset_force_sensor_calib_data (void);


#ifdef __cplusplus
}
#endif

#endif // ! MOTOR_ALGO_H
