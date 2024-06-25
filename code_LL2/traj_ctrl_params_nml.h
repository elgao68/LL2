/////////////////////////////////////////////////////////////////////////////
//
// traj_ctrl_params_nml.h.h
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LL2_TRAJ_CTRL_PARAMS_NML_H_
#define LL2_TRAJ_CTRL_PARAMS_NML_H_

#include <_coord_system.h>
#include <nml.h>
#include <nml_util.h>
#include <stdbool.h>
#include <math.h>

#include "_std_c.h"

typedef struct {
	float cycle_period;
	float exp_blend_time;
	float semiaxis_x;
	float semiaxis_y;
	float rot_angle;
	bool cycle_dir; //false = CW; true = CCW
} traj_ctrl_params_t;

////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY CONTROL PARAMS STRUCT - GAO
////////////////////////////////////////////////////////////////////////////////

traj_ctrl_params_t
set_traj_ctrl_params(	float cycle_period, float exp_blend_time,
						float semiaxis_x, float semiaxis_y,
						float rot_angle, bool cycle_dir);

////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY FUNCTIONS - ELLIPTICAL:
////////////////////////////////////////////////////////////////////////////////

void
traj_ellipse_points(	double phi, double dt_phi, double p[], double dt_p[], double u_t[],
						double a_x, double a_y, double ang);

void
traj_ellipse_constraints(	double phi, double dt_phi, nml_mat* A_con, nml_mat* b_con,
							double a_x, double a_y, double ang);

void
traj_ellipse_help(	double phi, double dt_phi, double p[], double dt_p[], double u_t[],
					nml_mat* A_con, nml_mat* b_con,
					double a_x, double a_y, double ang);

////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY FUNCTIONS - LINEAR:
////////////////////////////////////////////////////////////////////////////////

void
traj_linear_points(	double p[], double dt_p[], double u_t[], double dt_k,
					double p_o[], double p_f[], double v_max, double alpha, uint8_t* initial);

void
pos_linear_relative(double* dist_rel, double* v_rel, double t, double dist_max, double v_max, double alpha);

////////////////////////////////////////////////////////////////////////////////
// ANCILLARY FUNCTIONS - GAO
////////////////////////////////////////////////////////////////////////////////

void
rotate_vect_2d(double v_f[], double v_o[], double ang);

int
sign_force_tang_switch(double phi);

#endif
