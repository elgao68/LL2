/////////////////////////////////////////////////////////////////////////////
//
// motor_algo_ll2.h
//
// Created on: 2024.06.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef  MOTOR_ALGO_H
#define  MOTOR_ALGO_H

#include <_coord_system.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <control_settings_ll2.h>
#include <dynamic_systems_nml.h>
#include <lowerlimb_config.h>
#include <traj_ctrl_params_nml.h>
#include <qei_motor_drivers_LL2.h>
#include "transition_mode.h"

#ifdef __cplusplus
	extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////
// CONTROL TIME STEP
///////////////////////////////////////////////////////////////////////////////

#define DT_STEP_MSEC   5
#define MSEC_PER_SEC   1000

///////////////////////////////////////////////////////////////////////////////
// Dynamic system parameters:
///////////////////////////////////////////////////////////////////////////////

extern admitt_model_params_t admitt_model_params_local;

///////////////////////////////////////////////////////////////////////////////
// INTEGRATOR PARAMETERS:
///////////////////////////////////////////////////////////////////////////////

#define N_PREV_INT_STEPS 2

// Time delay indices:
#define DELAY_1	    0
#define DELAY_2	    1

#define SWITCH_TRAJ_START	 1
#define SWITCH_TRAJ_END		-1
#define SWITCH_TRAJ_NULL     0

#define TRAJ_PARAMS_GROW	 1
#define TRAJ_PARAMS_DECAY	-1
#define TRAJ_PARAMS_STEADY	 0

///////////////////////////////////////////////////////////////////////////////
// Display variables:
///////////////////////////////////////////////////////////////////////////////

#define DT_DISP_MSEC_ALGO		 1000
#define DT_DISP_MSEC_CALIB		 1000

#define USE_ITM_OUT_TRAJ_REF		1
#define USE_ITM_OUT_ADMITT_MODEL	1
#define USE_ITM_OUT_CALIB_CHECK		1

///////////////////////////////////////////////////////////////////////////////
// TYPE DEFINITIONS:
///////////////////////////////////////////////////////////////////////////////

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
    float current;				//calc current flow in Amperes
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

///////////////////////////////////////////////////////////////////////////////
// ODE PARAMETER INDICES:
///////////////////////////////////////////////////////////////////////////////

#define IDX_PAR_NML_A_CON 	0
#define IDX_PAR_NML_B_CON 	1

///////////////////////////////////////////////////////////////////////////////
// MOTION ALGORITHMS - ACTIVE:
///////////////////////////////////////////////////////////////////////////////

// ELLIPTIC TRAJECTORY - ACTIVE:
void traj_ref_step_active_elliptic(
		double p_ref[],	double dt_p_ref[],
		double* phi_ref, double* dt_phi_ref,
		double u_t_ref[], double dt_k, double F_end_in[], double z_intern_o_dbl[],
		traj_ctrl_params_t traj_ctrl_params, admitt_model_params_t admitt_model_params, int use_admitt_model_constr, int8_t switch_traj, int8_t use_traj_params_variable);

///////////////////////////////////////////////////////////////////////////////
// MOTION ALGORITHMS - PASSIVE:
///////////////////////////////////////////////////////////////////////////////

// ELLIPTIC TRAJECTORY - PASSIVE:
void traj_ref_step_passive_elliptic(
		double p_ref[],	double dt_p_ref[],
		double* phi_ref, double* dt_phi_ref,
		double u_t_ref[], double dt_k,
		traj_ctrl_params_t traj_ctrl_params, int8_t switch_traj, int8_t use_traj_params_variable);

// ISOMETRIC "TRAJECTORY":
void traj_ref_step_isometric(
		double p_ref[],	double dt_p_ref[],
		double* phi_ref, double* dt_phi_ref,
		double u_t_ref[]);

// CALIBRATION TRAJECTORY:
void traj_ref_calibration_ll2(
	double p_ref[], double dt_p_ref[], uint8_t* calib_enc_on, calib_traj_t* calib_traj, uint8_t* idx_scale, double z_intern_o_dbl[],
	double dt_k, double p_m[], double dt_p_m[], double phi_o, double dt_phi_o,
	lowerlimb_motors_settings_t* LL_motors_settings, traj_ctrl_params_t* traj_ctrl_params, uint8_t traj_exerc_type, double v_calib, double frac_ramp_calib);

// HOMING TRAJECTORY:
void
traj_ref_homing_ll2(double p_ref[], double dt_p_ref[], uint8_t* home_traj_on, uint8_t* init_home_traj, uint8_t* idx_scale,
	double dt_k, double p_m[], double dt_p_m[], double phi_o, double dt_phi_o,
	traj_ctrl_params_t* traj_ctrl_params, double v_calib, double frac_ramp_calib);

///////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS, DYNAMIC SYSTEMS
///////////////////////////////////////////////////////////////////////////////

void ode_admitt_model_nml(nml_mat* dt_z_nml, nml_mat* z, ode_param_struct ode_params);

void admitt_model_matrices_nml(nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, double m_val[], double b_val[], double k_val[], size_t N_q);

///////////////////////////////////////////////////////////////////////////////
// LOWER-LIMB UTILITY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

uint8_t set_LL_mech_readings(lowerlimb_mech_readings_t* mech, uint64_t current_upTime,
		int32_t L_count, int32_t R_count,
		uint32_t L_currsens, uint32_t R_currsens,
		uint32_t force_end_in_x, uint32_t force_end_in_y,
		uint8_t isCalibration);

void set_LL_motor_settings(lowerlimb_motors_settings_t* motors, float force_end[]);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS, LOW LEVEL
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Safety functions:
///////////////////////////////////////////////////////////////////////////////

void set_safetyOff(uint8_t en);
uint8_t get_safetyOff(void);

///////////////////////////////////////////////////////////////////////////////
// Sensor readings:
///////////////////////////////////////////////////////////////////////////////

void clear_lowerlimb_mech_readings(lowerlimb_mech_readings_t* LL_mech_readings);

///////////////////////////////////////////////////////////////////////////////
// Motor settings:
///////////////////////////////////////////////////////////////////////////////

void clear_lowerlimb_motors_settings(lowerlimb_motors_settings_t* LL_motors_settings);
uint32_t get_motor_L_dac(lowerlimb_motors_settings_t* LL_motors_settings);
uint32_t get_motor_R_dac(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_L_direction(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_R_direction(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_L_enable(lowerlimb_motors_settings_t* LL_motors_settings);
bool get_motor_R_enable(lowerlimb_motors_settings_t* LL_motors_settings);

///////////////////////////////////////////////////////////////////////////////
// Exercise functions:
///////////////////////////////////////////////////////////////////////////////

void init_motor_algo(lowerlimb_mech_readings_t* LL_mech_readings, lowerlimb_motors_settings_t* LL_motors_settings);
uint8_t lowerlimb_motor_safety_limiter(uint8_t type, uint8_t targ_index);

///////////////////////////////////////////////////////////////////////////////
// Offset forces:
///////////////////////////////////////////////////////////////////////////////

void set_force_sensor_zero_offset (float X_axis, float Y_axis);

#ifdef __cplusplus
}
#endif

#endif // ! MOTOR_ALGO_H
