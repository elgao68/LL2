/////////////////////////////////////////////////////////////////////////////
//
// traj_ctrl_params_nml.c
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <control_settings_ll2.h>
#include <traj_ctrl_params_nml.h>

////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY CONTROL PARAMS STRUCT
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY FUNCTIONS - ELLIPTICAL:
////////////////////////////////////////////////////////////////////////////////

void
traj_ellipse_points(	double phi, double dt_phi, double p[], double dt_p[], double u_t[],
						double a_x, double a_y, double ang) {

	///////////////////////////////////////////////////////////////////////////////
	// Declare static matrices:
	///////////////////////////////////////////////////////////////////////////////

	// Dummy matrices:
	static nml_mat* A_con_dum;
	static nml_mat* b_con_dum;

	///////////////////////////////////////////////////////////////////////////////
	// Initialize matrices:
	///////////////////////////////////////////////////////////////////////////////

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
	double  v_tang_o[N_COORD_2D];

	// Position:
	p_o[IDX_X] = a_x*cos(phi);
	p_o[IDX_Y] = a_y*sin(phi);

	// Tangential unit vector:
	v_tang_o[IDX_X] = -a_x*sin(phi);
	v_tang_o[IDX_Y] =  a_y*cos(phi);

	double magn_v_tang_o = pow(pow(v_tang_o[IDX_X],2) + pow(v_tang_o[IDX_Y],2), 0.5);

	if (magn_v_tang_o > 0) {
		u_t_o[IDX_X] = v_tang_o[IDX_X] / magn_v_tang_o;
		u_t_o[IDX_Y] = v_tang_o[IDX_Y] / magn_v_tang_o;
	}
	else if (a_x != 0 && a_y == 0) {
		u_t_o[IDX_X] = 1.0;
		u_t_o[IDX_Y] = 0.0;
	}
	else if (a_x == 0 && a_y != 0) {
		u_t_o[IDX_X] = 0.0;
		u_t_o[IDX_Y] = 1.0;
	}
	else {
		u_t_o[IDX_X] = 0.0;
		u_t_o[IDX_Y] = 0.0;
	}

	// Velocity:
	dt_p_o[IDX_X] = dt_phi*v_tang_o[IDX_X];
	dt_p_o[IDX_Y] = dt_phi*v_tang_o[IDX_Y];

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

////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY FUNCTIONS - LINEAR:
////////////////////////////////////////////////////////////////////////////////

void
traj_linear_points(	double p[], double dt_p[], double u_t[], double dt_k,
					double p_o[], double p_f[], double v_max, double frac_ramp_o, uint8_t* initial, double* T_f_ref,
					double* pos_rel, double* dt_pos_rel) {

	static int    step_int = 0; // relative time reference
	static double frac_ramp;
	static double dist_max;
	int c_i;

	if (*initial) {
		step_int = 0;

		// Correct ramp-up period:
		frac_ramp = frac_ramp_o;

		if (frac_ramp < 0)
			frac_ramp = 0;
		else if (frac_ramp > 0.5)
			frac_ramp = 0.5;

		// Distance between points:
		dist_max = pow(
				pow(p_f[IDX_X] - p_o[IDX_X],2) +
				pow(p_f[IDX_Y] - p_o[IDX_Y],2), 0.5);

		*initial = 0;
	}

	// Reference time:
	double t = dt_k*step_int;

	// Tangential unit vector:
	for (int c_i = 0; c_i < N_COORD_2D; c_i++)
		if (dist_max > 0)
			u_t[c_i] = (p_f[c_i] - p_o[c_i])/dist_max;
		else
			u_t[IDX_X] = 0;

	// Compute relative position and velocity:
	pos_linear_relative(pos_rel, dt_pos_rel, t, dist_max, v_max, frac_ramp, T_f_ref);

	// Compute absolute position and velocity:
	for (int c_i = 0; c_i < N_COORD_2D; c_i++) {
		p[c_i] = p_o[c_i] + (*pos_rel)*u_t[c_i];

		dt_p[c_i] = (*dt_pos_rel)*u_t[c_i];
	}

	// Increase step counter:
	step_int++;
}

void
pos_linear_relative(double* pos_rel, double* dt_pos_rel, double t, double dist_max, double v_max, double frac_ramp, double* T_f_ref) {
	double T_f;

	if (dist_max > 0 && v_max > 0) {
		// Final time (relative to t = 0):
		T_f = dist_max/(1 - frac_ramp)/v_max;

		// Acceleration (ramp stage):
		double a_ramp = v_max/frac_ramp/T_f;

		// "Via point" time:
		double t1;

		//////////////////////////////////////////////////////////////
		// Compute position and velocity:
		//////////////////////////////////////////////////////////////

		// Safety catch:
		if (t < 0) {
			*dt_pos_rel    = 0.0;
			*pos_rel = 0.0;
		}
		// Ramp-up:
		else if (t >= 0 && t < frac_ramp*T_f) {
			*dt_pos_rel    = a_ramp*t;
			*pos_rel = 1/2.0*a_ramp*pow(t, 2);
		}
		// Constant velocity:
		else if (t >= frac_ramp*T_f && t <= (1 - frac_ramp)*T_f) {
			t1 = t - frac_ramp*T_f;

			*dt_pos_rel    = v_max;
			*pos_rel = 1/2.0*a_ramp*pow(frac_ramp*T_f, 2) + v_max*t1;
		}
		// Ramp-down:
		else if (t > (1 - frac_ramp)*T_f && t <= T_f) {
			t1 = t - (1 - frac_ramp)*T_f;

			*dt_pos_rel    = a_ramp*(frac_ramp*T_f - t1);
			*pos_rel = 1/2.0*a_ramp*pow(frac_ramp*T_f, 2) + v_max*(1 - 2*frac_ramp)*T_f +
							a_ramp*(frac_ramp*T_f*t1 - 1/2.0*pow(t1, 2));
		}
		// Final position:
		else {
			*dt_pos_rel    = 0.0;
			*pos_rel = dist_max;
		}
	}
	else {
		T_f = 0;
		*dt_pos_rel    = 0.0;
		*pos_rel = 0.0;
	}

	*T_f_ref = T_f;
}


////////////////////////////////////////////////////////////////////////////////
// ANCILLARY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void
rotate_vect_2d(double v_f[], double v_o[], double ang) {

	v_f[IDX_X] = v_o[IDX_X]*cos(ang) - v_o[IDX_Y]*sin(ang);
	v_f[IDX_Y] = v_o[IDX_X]*sin(ang) + v_o[IDX_Y]*cos(ang);
}

int
sign_force_tang_switch(double phi) {
	static double ERR_PHI_RAD = 0.2;
	static int sign_F_tang    = 1;
	static int switch_armed   = 1;

	double phi_wr = fmod(phi, 2*PI); // phase wrap-around value
	static double phi_wr_prev;

	// Sign switching condition:
	if ((fabs(phi_wr) < ERR_PHI_RAD || fabs(phi_wr) > PI - ERR_PHI_RAD) && switch_armed) {
		sign_F_tang = -sign_F_tang;
		switch_armed = 0;

		printf("\n");
		printf("sign_F_tang = [%d]\n\n", sign_F_tang);
	}
	// Switch arming condition:
	else if (((phi_wr - PI/2)*(phi_wr_prev - PI/2) <= 0 ||
			  (phi_wr + PI/2)*(phi_wr_prev + PI/2) <= 0 ) &&
			  !switch_armed) {
				switch_armed = 1;

				printf("\n");
				printf("switch_armed = [%d]\n\n", switch_armed);
	}

	phi_wr_prev = phi_wr;

	return sign_F_tang;
}

