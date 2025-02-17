/////////////////////////////////////////////////////////////////////////////
//
// motor_algo_ll2_active.c
//
// Created on: 2024.06.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <motor_algo_ll2.h>

///////////////////////////////////////////////////////////////////////////////
// Dynamic system parameters:
///////////////////////////////////////////////////////////////////////////////

extern admitt_model_params_t admitt_model_params_local;

///////////////////////////////////////////////////////////////////////////////
// ELLIPTICAL TRAJECTORY - ACTIVE:
///////////////////////////////////////////////////////////////////////////////

void
traj_ref_step_active_elliptic(
	double z_intern_home_dbl[],
	double p_ref[],	double dt_p_ref[],
	double* phi_ref, double* dt_phi_ref,
	double u_t_ref[], double dt_k, double F_end_in[],
	traj_ctrl_params_t traj_ctrl_params, admitt_model_params_t admitt_model_params, int use_admitt_model_constr, int8_t mode_traj, int8_t use_traj_params_variable, uint8_t* init_traj) {

    ///////////////////////////////////////////////////////////////////////////////
    // Declare static matrices:
    ///////////////////////////////////////////////////////////////////////////////

	// Admittance model state:
	static nml_mat* z_intern;
	static nml_mat* z_intern_prev[N_PREV_INT_STEPS];

	// Trajectory path - constraint matrices:
	static nml_mat* A_con;
	static nml_mat* b_con;

	///////////////////////////////////////////////////////////////////////////////
	// Dynamic system (ODE) parameters:
	///////////////////////////////////////////////////////////////////////////////

	static ode_param_struct ode_params;

	ode_params.USE_ODE_CONSTR = use_admitt_model_constr;
	ode_params.DIM = 2*N_COORD_EXT;

	///////////////////////////////////////////////////////////////////////////////
	// Tangential (assistive / resistive) force:
	///////////////////////////////////////////////////////////////////////////////

	double F_tang[N_COORD_2D]   = {0.0, 0.0};

	///////////////////////////////////////////////////////////////////////////////
	// Internal model kinematics (needed for tangential force computation):
	///////////////////////////////////////////////////////////////////////////////

	double    p_intern[N_COORD_2D] = {0.0, 0.0};
	double dt_p_intern[N_COORD_2D] = {0.0, 0.0};
	double  u_t_intern[N_COORD_2D] = {0.0, 0.0};

	///////////////////////////////////////////////////////////////////////////////
    // Counters:
    ///////////////////////////////////////////////////////////////////////////////

	int c_i;

	///////////////////////////////////////////////////////////////////////////////
    // Initialize nml matrices:
    ///////////////////////////////////////////////////////////////////////////////

	static int init_nml = 1;

	if (init_nml) {
		// Admittance model state:
		z_intern = nml_mat_new(2*N_COORD_EXT, 1); // internal state

		for (c_i = 0; c_i < N_PREV_INT_STEPS; c_i++)
			z_intern_prev[c_i] = nml_mat_new(2*N_COORD_EXT, 1); // internal state

		// Trajectory path - constraint matrices:
		A_con = nml_mat_new(N_CONSTR_TRAJ, N_COORD_EXT);
		b_con = nml_mat_new(N_CONSTR_TRAJ, 1);

		ode_params.par_nml[IDX_PAR_NML_A_CON] = nml_mat_new(N_CONSTR_TRAJ, N_COORD_EXT);

		init_nml = 0;
	}

	///////////////////////////////////////////////////////////////////////////////
	// COPY ADMITTANCE PARAMETERS TO LOCAL SCOPE VARIABLES:
	///////////////////////////////////////////////////////////////////////////////

    memcpy(&admitt_model_params_local, &admitt_model_params, sizeof(admitt_model_params_t));

	///////////////////////////////////////////////////////////////////////////////
	// COPY TRAJECTORY PARAMETERS TO LOCAL SCOPE VARIABLES:
	///////////////////////////////////////////////////////////////////////////////

	// double T_cycle = traj_ctrl_params.cycle_period;
	double T_exp   = traj_ctrl_params.exp_blend_time;
	double ax_x    = traj_ctrl_params.semiaxis_x;
	double ax_y    = traj_ctrl_params.semiaxis_y;
	double rot_ang = traj_ctrl_params.rot_angle;
	int cycle_dir  = (int)traj_ctrl_params.cycle_dir;

	double sig_exp = 3.0/T_exp;
	int sgn_F_tang;

	float F_tang_magn = admitt_model_params.F_tang_magn;

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
	// STATIC VARIABLES declarations:
	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
    // Sensitivity to sensor force inputs:
    ///////////////////////////////////////////////////////////////////////////////

	static double frac_F_end_in = 1.0;

	///////////////////////////////////////////////////////////////////////////////
	// Variable trajectory parameters:
	///////////////////////////////////////////////////////////////////////////////

	static double ax_x_adj;
	static double ax_y_adj;

	///////////////////////////////////////////////////////////////////////////////
	// Additional modes & behaviors:
	///////////////////////////////////////////////////////////////////////////////

	static uint8_t is_linear_traj;  // special case: linear trajectory
	static int8_t mode_traj_prev = MODE_TRAJ_NULL; // previous trajectory mode (for switching detection)

	///////////////////////////////////////////////////////////////////////////////
	// Algorithm step counter:
	///////////////////////////////////////////////////////////////////////////////

	static int step_int = 0;

	///////////////////////////////////////////////////////////////////////////////
	// Trajectory initialization - initialize STATIC VARIABLES (CRITICAL):
	///////////////////////////////////////////////////////////////////////////////

	if (*init_traj) {

		// (Re)initialize force fraction:
		frac_F_end_in = 1.0;

		// Variable trajectory parameters:
		if (use_traj_params_variable) {
			ax_x_adj = 0;
			ax_y_adj = 0;
		}
		else {
			ax_x_adj = ax_x;
			ax_y_adj = ax_y;
		}

		// Is trajectory linear?
		if (ax_y == 0)
			is_linear_traj = 1;
		else
			is_linear_traj = 0;

		// Trajectory mode:
		mode_traj_prev = MODE_TRAJ_NULL;

		// (Re)initialize step counter (CRITICAL):
		step_int = 0;

		*init_traj = 0;
	}
	// END STATIC VARIABLES declarations

	///////////////////////////////////////////////////////////////////////////////
	// Timers:
	///////////////////////////////////////////////////////////////////////////////

	double t_ref = dt_k*step_int;

	static double t_param_o = 0;

    ///////////////////////////////////////////////////////////////////////////////
    //  Variable trajectory path parameters - behaviors:
    ///////////////////////////////////////////////////////////////////////////////

	if (use_traj_params_variable) {

		static double ax_x_o;
		static double ax_y_o;

		if (mode_traj == MODE_TRAJ_START && mode_traj != mode_traj_prev) {
			t_param_o = t_ref;
			ax_x_o = ax_x_adj;
			ax_y_o = ax_y_adj;
		}
		else if (mode_traj == MODE_TRAJ_END && mode_traj != mode_traj_prev) {
			t_param_o = t_ref;
			ax_x_o = ax_x_adj;
			ax_y_o = ax_y_adj;
		}

		if (mode_traj == MODE_TRAJ_START) {
			ax_x_adj = (1.0 - exp(-sig_exp*(t_ref - t_param_o)))*(ax_x - ax_x_o) + ax_x_o;
			ax_y_adj = (1.0 - exp(-sig_exp*(t_ref - t_param_o)))*(ax_y - ax_y_o) + ax_y_o;
		}
		else if (mode_traj == MODE_TRAJ_END) {
			ax_x_adj = exp(-sig_exp*(t_ref - t_param_o))*ax_x_o;
			ax_y_adj = exp(-sig_exp*(t_ref - t_param_o))*ax_y_o;
		}
	}

    ///////////////////////////////////////////////////////////////////////////////
    //  Fixed trajectory path parameters - behaviors:
    ///////////////////////////////////////////////////////////////////////////////

	else { // (!use_traj_params_variable)
		if (mode_traj == MODE_TRAJ_END && mode_traj != mode_traj_prev)
			t_param_o = t_ref;

		///////////////////////////////////////////////////////////////////////////////
		// Make input force decay:
		///////////////////////////////////////////////////////////////////////////////

		if (mode_traj == MODE_TRAJ_END)
			frac_F_end_in = exp(-sig_exp*(t_ref - t_param_o));
	}

    ///////////////////////////////////////////////////////////////////////////////
    // Integrate admittance model:
    ///////////////////////////////////////////////////////////////////////////////

	if (step_int == 0) {

		///////////////////////////////////////////////////////////////////////////////
		// Initialize nml internal state:
		///////////////////////////////////////////////////////////////////////////////

		for (c_i = 0; c_i < 2*N_COORD_EXT; c_i++)
			z_intern->data[c_i][0] = z_intern_home_dbl[c_i];
	}
	else {
		///////////////////////////////////////////////////////////////////////////////
		// Dynamic system (ODE) inputs and other parameters:
		///////////////////////////////////////////////////////////////////////////////

		// Obtain trajectory path constraint (only A_con will be used for now):
		if (use_admitt_model_constr) {

			// Obtain internal trajectory state (p_intern, dt_p_intern, u_t_intern) - needed to compute tangential force:
			traj_ellipse_help(*phi_ref, *dt_phi_ref, p_intern, dt_p_intern, u_t_intern, A_con, b_con, ax_x_adj, ax_y_adj, rot_ang);

			// Include constraint matrix in ODE parameters:
			nml_mat_cp_ref(ode_params.par_nml[IDX_PAR_NML_A_CON], A_con);

			// Tangential force sign:
			if (!is_linear_traj)
				sgn_F_tang = cycle_dir;
			else
				sgn_F_tang = sign_force_tang_switch(*phi_ref);

			// Compute tangential force:
			for (c_i = IDX_X; c_i <= IDX_Y; c_i++)
				F_tang[c_i] = sgn_F_tang*F_tang_magn*u_t_intern[c_i];
		}
		else {
			F_tang[IDX_X] = 0.0;
			F_tang[IDX_Y] = 0.0;
		}

		// Input forces:
		ode_params.par_dbl[IDX_X]   = frac_F_end_in*(F_end_in[IDX_X] + F_tang[IDX_X]);
		ode_params.par_dbl[IDX_Y]   = frac_F_end_in*(F_end_in[IDX_Y] + F_tang[IDX_Y]);
		ode_params.par_dbl[IDX_PHI] = 0.0;

		///////////////////////////////////////////////////////////////////////////////
		// Integration step for ODE:
		///////////////////////////////////////////////////////////////////////////////

		#if USE_ODE_SOLVER_RUNGE_KUTTA
			solve_ode_sys_rkutta_ord4_nml(z_intern, z_intern_prev[DELAY_1], dt_k, ode_admitt_model_nml, ode_params);
		#else
			solve_ode_sys_rectang_nml(z_intern, z_intern_prev[DELAY_1], dt_k, ode_admitt_model_nml, ode_params); // TODO: verify integrator robustness
		#endif
	}

	nml_mat_cp_ref(z_intern_prev[DELAY_1], z_intern);

    ///////////////////////////////////////////////////////////////////////////////
    // Obtain reference trajectory position and velocity:
    ///////////////////////////////////////////////////////////////////////////////

	if (use_admitt_model_constr) {
		// Extract phase and instantaneous frequency from INTERNAL STATE:
		*phi_ref    = z_intern->data[IDX_PHI][0];
		*dt_phi_ref = z_intern->data[IDX_DT_PHI][0];

		// Compute reference trajectory position and velocity from PHASE:
		traj_ellipse_points(*phi_ref, *dt_phi_ref, p_ref, dt_p_ref, u_t_ref,
			ax_x_adj, ax_y_adj, rot_ang);
	}
	else {
		// Phase and instantaneous frequency are not updated:
		*phi_ref    = 0.0;
		*dt_phi_ref = 0.0;

		// Extract reference trajectory position and velocity from INTERNAL STATE:
		p_ref[IDX_X]    = z_intern->data[IDX_X][0];
		p_ref[IDX_Y]    = z_intern->data[IDX_Y][0];

		dt_p_ref[IDX_X] = z_intern->data[IDX_DT_X][0];
		dt_p_ref[IDX_Y] = z_intern->data[IDX_DT_Y][0];
	}

    ///////////////////////////////////////////////////////////////////////////////
	// ITM console output:
    ///////////////////////////////////////////////////////////////////////////////

	#if USE_ITM_OUT_TRAJ_REF
		if (step_int == 0) {
			printf("   traj_ref_step_active_elliptic(): \n");
			// printf("   T_cycle = %f\n", T_cycle);
			printf("   T_exp   = %f\n", T_exp);
			printf("   ax_x    = %f\n", ax_x);
			printf("   ax_y    = %f\n", ax_y);
			printf("   rot_ang  = %f\n", rot_ang);
			printf("   cycle_dir = %f\n", (double)cycle_dir);
			printf("\n");
		}

		if (step_int % (DT_DISP_MSEC_ALGO/(int)(1000*dt_k)) == 0) {
			printf("[%d] phi = [%3.2f] \tdt_phi = [%3.2f] \tF_end_in = [%3.2f, %3.2f] \tfrac_F_end_in = [%3.2f] \tu_t_int = [%3.2f, %3.2f]\tF_tang = [%3.2f, %3.2f] \tz = [",
				step_int,
				*phi_ref, *dt_phi_ref,
				F_end_in[IDX_X], F_end_in[IDX_Y], frac_F_end_in,
				u_t_intern[IDX_X], u_t_intern[IDX_Y], F_tang[IDX_X], F_tang[IDX_Y]);

			for (int r_i = 0; r_i < 2*N_COORD_EXT; r_i++)
				printf("%3.2f ", z_intern->data[r_i][0]);

			printf("]\n\n");
		}
	#endif

	///////////////////////////////////////////////////////////////////////////////
	// Store current trajectory mode:
	///////////////////////////////////////////////////////////////////////////////

	mode_traj_prev = mode_traj;

	///////////////////////////////////////////////////////////////////////////////
	// Increase step counter:
	///////////////////////////////////////////////////////////////////////////////

	step_int++;
}
