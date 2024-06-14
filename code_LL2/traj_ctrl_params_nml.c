/////////////////////////////////////////////////////////////////////////////
//
// traj_ctrl_params_nml.h.c
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <control_settings_ll2.h>
#include <traj_ctrl_params_nml.h>

//////////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY CONTROL PARAMS STRUCT - GAO
//////////////////////////////////////////////////////////////////////////////////////

traj_ctrl_params_t
set_traj_ctrl_params(	float cycle_period, float exp_blend_time,
							float semiaxis_x, float semiaxis_y,
							float rot_angle, bool cycle_dir) {

	traj_ctrl_params_t traj_ctrl_params;

	traj_ctrl_params.cycle_period = cycle_period;
	traj_ctrl_params.exp_blend_time = exp_blend_time;
	traj_ctrl_params.semiaxis_x = semiaxis_x;
	traj_ctrl_params.semiaxis_y = semiaxis_y;
	traj_ctrl_params.rot_angle = rot_angle;
	traj_ctrl_params.cycle_dir = cycle_dir;

	return traj_ctrl_params;
}

//////////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY FUNCTIONS - VARIOUS - GAO
//////////////////////////////////////////////////////////////////////////////////////

void
traj_ellipse_points(	double phi, double dt_phi, double p[], double dt_p[], double u_t[],
						double a_x, double a_y, double ang) {

	/////////////////////////////////////////////////////////////////////////////////////
	// Declare static matrices:
	/////////////////////////////////////////////////////////////////////////////////////

	// Dummy matrices:
	static nml_mat* A_con_dum;
	static nml_mat* b_con_dum;

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize matrices:
	/////////////////////////////////////////////////////////////////////////////////////

	static int initial = 1;

	if (initial) {
		A_con_dum = nml_mat_new(N_CONSTR_TRAJ, N_COORD_EXT);
		b_con_dum = nml_mat_new(N_CONSTR_TRAJ, 1);

		initial = 0;
	}

	nml_mat_all_set(A_con_dum, 0);
	nml_mat_all_set(b_con_dum, 0);

	traj_ellipse_help(phi, dt_phi, p, dt_p, u_t, A_con_dum, b_con_dum,
		a_x, a_y, ang);
}

void
traj_ellipse_constraints(	double phi, double dt_phi, nml_mat* A_con, nml_mat* b_con,
							double a_x, double a_y, double ang) {

	// Dummy vectors:
	double    p_dum[N_COORD_2D];
	double dt_p_dum[N_COORD_2D];
	double  u_t_dum[N_COORD_2D];

	traj_ellipse_help(phi, dt_phi, p_dum, dt_p_dum, u_t_dum, A_con, b_con,
		a_x, a_y, ang);
}

void
traj_ellipse_help(	double phi, double dt_phi, double p[], double dt_p[], double u_t[],
					nml_mat* A_con, nml_mat* b_con,
					double a_x, double a_y, double ang) {

	// Base values:
	double    p_o[N_COORD_2D];
	double dt_p_o[N_COORD_2D];
	double  u_t_o[N_COORD_2D];

	// Position:
	p_o[IDX_X] = a_x*cos(phi);
	p_o[IDX_Y] = a_y*sin(phi);

	// Velocity:
	dt_p_o[IDX_X] = -dt_phi*a_x*sin(phi);
	dt_p_o[IDX_Y] =  dt_phi*a_y*cos(phi);

	// Tangential unit vector:
	double magn_dt_p = pow(pow(dt_p_o[IDX_X],2) + pow(dt_p_o[IDX_Y],2), 0.5);

	if (magn_dt_p > 0) {
		u_t_o[IDX_X] = dt_p_o[IDX_X] / magn_dt_p;
		u_t_o[IDX_Y] = dt_p_o[IDX_Y] / magn_dt_p;
	}
	else {
		u_t_o[IDX_X] = 0.0;
		u_t_o[IDX_Y] = 0.0;
	}

	// Rotate vectors:
	rotate_vect_2d(p,    p_o,    ang);
	rotate_vect_2d(dt_p, dt_p_o, ang);
	rotate_vect_2d(u_t,  u_t_o,  ang);

	// Constraint matrices:
	A_con -> data[IDX_X][0] = 1.0;
	A_con -> data[IDX_X][1] = 0.0;
	A_con -> data[IDX_X][2] = a_x*cos(ang)*sin(phi) + a_y*cos(phi)*sin(ang) ;

	A_con -> data[IDX_Y][0] = 0.0;
	A_con -> data[IDX_Y][1] = 1.0;
	A_con -> data[IDX_Y][2] = a_x*sin(ang)*sin(phi) - a_y*cos(ang)*cos(phi);

	b_con -> data[IDX_X][0] = -pow(dt_phi,2)*(a_x*cos(ang)*cos(phi) - a_y*sin(ang)*sin(phi));
	b_con -> data[IDX_Y][0] = -pow(dt_phi,2)*(a_x*cos(phi)*sin(ang) + a_y*cos(ang)*sin(phi));
}

//////////////////////////////////////////////////////////////////////////////////////
// ANCILLARY FUNCTIONS - GAO
//////////////////////////////////////////////////////////////////////////////////////

void
rotate_vect_2d(double v_f[], double v_o[], double ang) {

	v_f[IDX_X] = v_o[IDX_X]*cos(ang) - v_o[IDX_Y]*sin(ang);
	v_f[IDX_Y] = v_o[IDX_X]*sin(ang) + v_o[IDX_Y]*cos(ang);
}

