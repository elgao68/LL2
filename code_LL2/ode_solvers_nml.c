/////////////////////////////////////////////////////////////////////////////
//
// ode_solvers_nml.c
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <ode_solvers_nml.h>

#define DECIM_DISP_ODE	  400
#define USE_ITM_OUT_ODE		1

#define ORD_4	4

void
solve_ode_sys_rkutta_ord4_nml(nml_mat* y_f, nml_mat* y_o, double T, nml_mat* (*ode_func)(nml_mat*, ode_param_struct), ode_param_struct ode_params) {

	// Time integration step for an n-dimensional system of ODEs using 4th-order Runge-Kutta

	double F[] = {    0, 1/2.0, 1/2.0, 1.0}; // time step array
	double G[] = {1/6.0, 1/3.0, 1/3.0, 1/6.0}; // weights array

	/////////////////////////////////////////////////////////////////////////////////////
	// Declare static matrices:
	/////////////////////////////////////////////////////////////////////////////////////

	static nml_mat* K[ORD_4];
	static nml_mat* FK;
	static nml_mat* K_scaled;
	static nml_mat* y_step;
	static nml_mat* GK;
	static nml_mat* sum_K_weighted;

	// Counters:
	int i;

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize matrices:
	/////////////////////////////////////////////////////////////////////////////////////

	static int initial = 1;

	if (initial) {
		for (i = 0; i < ORD_4; i++)
			K[i] = nml_mat_new(ode_params.DIM, 1);

		FK           = nml_mat_new(ode_params.DIM, 1);
		K_scaled     = nml_mat_new(ode_params.DIM, 1);
		y_step       = nml_mat_new(ode_params.DIM, 1);
		GK           = nml_mat_new(ode_params.DIM, 1);
		sum_K_weighted = nml_mat_new(ode_params.DIM, 1);

		initial = 0;
	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Compute "scaled" first derivatives at intermediate time points between 0 and T:
	/////////////////////////////////////////////////////////////////////////////////////

	for (i = 0; i < ORD_4; i++)
		if (i == 0) {
			K[0] = (*ode_func)(y_o, ode_params);
			nml_mat_smult_r(K[0], T);
		}
		else {
			// F(i)*K(i-i):
			nml_mat_cp_ref(FK, K[i - 1]);
			nml_mat_smult_r(FK, F[i]);

			// y_step(i) = y_o + F(i)*K(i-i):
			nml_mat_cp_ref(y_step, y_o);
			nml_mat_add_r(y_step, FK);

			// K_scaled(i) = f(y_step(i)):
			K_scaled = (*ode_func)(y_step, ode_params);

			// K(i) = T*K_scaled(i):
			nml_mat_cp_ref(K[i], K_scaled);
			nml_mat_smult_r(K[i], T);
		}

	/////////////////////////////////////////////////////////////////////////////////////
	// Compute weighted sum of "scaled" first derivatives:
	/////////////////////////////////////////////////////////////////////////////////////

	nml_mat_all_set(sum_K_weighted, 0.0); // initialize sum

	for (i = 0; i < ORD_4; i++) {
		// GK(i) = G(i)*K(i):
		nml_mat_cp_ref( GK, K[i]);
	    nml_mat_smult_r(GK, G[i]);

		nml_mat_add_r(sum_K_weighted, GK);
	}

    // Compute ODE output:
	nml_mat_cp_ref(y_f, y_o);
	nml_mat_add_r( y_f, sum_K_weighted);

	// ITM console output:
#if USE_ITM_OUT_ODE
	static int step_ode = 0;
	int c_i;

	if ((step_ode % DECIM_DISP_ODE) == 0) {
		printf("solve_ode_sys_rkutta_ord4_nml [%d]: T = [%f]\tF_end = [%f, %f]\n",
			step_ode, T, ode_params.par_dbl[IDX_X], ode_params.par_dbl[IDX_Y]);

		printf("y_o = [");
		for (c_i = 0; c_i < ode_params.DIM; c_i++) {
			printf("%f\t", y_o->data[c_i][0]);
		}
		printf("]\n");

		printf("____________________________\n");
		printf("sum_K_weighted = [");
		for (c_i = 0; c_i < ode_params.DIM; c_i++) {
			printf("%f\t", sum_K_weighted->data[c_i][0]);
		}
		printf("]\n");

		printf("____________________________\n");
		printf("y_f = [");
		for (c_i = 0; c_i < ode_params.DIM; c_i++) {
			printf("%f\t", y_f->data[c_i][0]);
		}
		printf("]\n");

		printf("\n");
	}

	// Increase counter:
	step_ode++;
#endif
}

/////////////////////////////////////////////////////////////////////////////
// ODE TEMPLATE:
/////////////////////////////////////////////////////////////////////////////

/*
nml_mat*
_ode_nml(nml_mat* x, ode_param_struct ode_params) {
	nml_mat* dt_x_m = nml_mat_new(ode_params.DIM, 1); // NOTE: make sure ode_params.ORD matches solver order

	return dt_x_m;
}
*/
