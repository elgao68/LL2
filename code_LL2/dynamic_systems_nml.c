/////////////////////////////////////////////////////////////////////////////
//
// dynamic_systems_nml.c
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <dynamic_systems_nml.h>

#define USE_ITM_OUT_DYN_SYS_MATR	0

void
dyn_sys_msd_nml_unc(nml_mat* dt_z_unc, nml_mat* Q_in, nml_mat* z, nml_mat* u_in,
		nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, nml_mat* q_eq) { // A_con

	size_t N_z = z->num_rows;
	size_t N_q = N_z/2;

    /////////////////////////////////////////////////////////////////////////////////////
    // Declare static matrices:
    /////////////////////////////////////////////////////////////////////////////////////

	static nml_mat* q;
	static nml_mat* v;

	static nml_mat* K_q_prod;
	static nml_mat* B_v_prod;

	static nml_mat* inv_M_sys;
	static nml_mat* inv_M_Q_dbl;

	/////////////////////////////////////////////////////////////////////////////////////
    // Initialize matrices:
    /////////////////////////////////////////////////////////////////////////////////////

	static int init_nml = 1;

	if (init_nml) {
		q = nml_mat_new(N_q, 1);
		v = nml_mat_new(N_q, 1);

		K_q_prod    = nml_mat_new(N_q, 1);
		B_v_prod    = nml_mat_new(N_q, 1);

		inv_M_sys   = nml_mat_new(N_q, N_q);
		inv_M_Q_dbl = nml_mat_new(N_q, 1);

		init_nml = 0;
	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Position and velocity vectors:
	/////////////////////////////////////////////////////////////////////////////////////

	int c_i, r_i;

	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++) {
		q->data[c_i][0] = z->data[c_i][0] - q_eq->data[c_i][0]; // offset position by equilibrium value
		v->data[c_i][0] = z->data[c_i + N_q][0];
	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Unconstrained dynamics:
	/////////////////////////////////////////////////////////////////////////////////////

	// Dynamics model: Q_in = -K_sys*(q - q_eq) - B_sys*v + u_in;

	nml_mat_dot_ref(K_q_prod, K_sys, q);
	nml_mat_dot_ref(B_v_prod, B_sys, v);

	// Unconstrained input term:
	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++)
		Q_in->data[c_i][0] = -K_q_prod->data[c_i][0] - B_v_prod->data[c_i][0] + u_in->data[c_i][0];

	// ITM console output:
#if USE_ITM_OUT_DYN_SYS_MATR
	static int step_i = 0;
	static int DECIM_DISP_UNC = DECIM_DISP_GENERAL;

	if ((step_i % DECIM_DISP_UNC) == 0) {
		printf("____________________________\n");
		printf("dyn_sys_msd_nml_unc [%d]:\n", step_i);

		printf("K_q_prod:\n");
		for (r_i = 0; r_i < K_q_prod->num_rows; r_i++) {
			for (c_i = 0; c_i < K_q_prod->num_cols; c_i++)
				printf("%f\t", K_q_prod->data[r_i][c_i]);
			printf("\n");
		}
		printf("\n");

		printf("B_v_prod:\n");
		for (r_i = 0; r_i < B_v_prod->num_rows; r_i++) {
			for (c_i = 0; c_i < B_v_prod->num_cols; c_i++)
				printf("%f\t", B_v_prod->data[r_i][c_i]);
			printf("\n");
		}
		printf("\n");

		printf("Q_in: ");
		for (c_i = IDX_X; c_i <= IDX_PHI; c_i++)
			printf("%f\t", Q_in->data[c_i][0]);
		printf("\n\n");
	}

	step_i++;
#endif

	/////////////////////////////////////////////////////////////////////////////////////
	// Unconstrained system output - computation:
	/////////////////////////////////////////////////////////////////////////////////////

	for (c_i = 0; c_i < M_sys->num_cols; c_i++)
		inv_M_sys->data[c_i][c_i] = 1.0/M_sys->data[c_i][c_i];

	nml_mat_dot_ref(inv_M_Q_dbl, inv_M_sys, Q_in);

	// Unconstrained output vector:
	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++) {
		dt_z_unc->data[c_i][0]       =           v->data[c_i][0];
		dt_z_unc->data[c_i + N_q][0] = inv_M_Q_dbl->data[c_i][0];
	}

}

void
dyn_sys_msd_nml_constr_lagr(nml_mat* dt_z_con, nml_mat* Q_in, nml_mat* z, nml_mat* u_in,
		nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, nml_mat* q_eq, nml_mat* A_con) {

	size_t N_z = z->num_rows;
	size_t N_q = N_z/2;
	size_t N_c = A_con->num_rows; // A_con is [N_c x N_q];

	// Counters:
	int r_i, c_i, r_i_loc, c_i_loc;

    /////////////////////////////////////////////////////////////////////////////////////
    // Declare static matrices:
    /////////////////////////////////////////////////////////////////////////////////////

	static nml_mat* v;
	static nml_mat* dt_z_unc;

	static nml_mat* A_con_tr;
	static nml_mat* zero_m;

	static nml_mat* mu;

	static nml_mat* L;
	static nml_mat* U;
	static nml_mat* P;
	// static nml_mat_lup* mu_LUP;
	unsigned int num_permutations;

	static nml_mat *dt_v_lam;
	static nml_mat* Q_in_ext;

	static nml_mat* P_Q_in_ext_prod;
	static nml_mat* y_dt_v_lam;

	/////////////////////////////////////////////////////////////////////////////////////
    // Initialize matrices:
    /////////////////////////////////////////////////////////////////////////////////////

	static int init_nml = 1;

	if (init_nml) {
		v        = nml_mat_new(N_q, 1);
		dt_z_unc = nml_mat_new(N_z, 1);

		A_con_tr = nml_mat_new(N_q, N_c);
		zero_m   = nml_mat_new(N_c, N_c);

		mu = nml_mat_new(N_q + N_c, N_q + N_c);

		// mu_LUP = malloc(sizeof(*mu_LUP)); // verify this
		L  = nml_mat_new(mu->num_rows, mu->num_rows);
		U  = nml_mat_new(mu->num_rows, mu->num_rows);
		P  = nml_mat_new(mu->num_rows, mu->num_rows);

		dt_v_lam = nml_mat_new(N_q + N_c, 1);
		Q_in_ext = nml_mat_new(N_q + N_c, 1);

		P_Q_in_ext_prod = nml_mat_new(P->num_rows, Q_in_ext->num_cols);
		y_dt_v_lam = nml_mat_new(U->num_cols, 1);

		init_nml = 0;
	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Velocity vector:
	/////////////////////////////////////////////////////////////////////////////////////

	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++)
		v->data[c_i][0] = z->data[c_i + N_q][0];

	/////////////////////////////////////////////////////////////////////////////////////
	// Unconstrained system output:
	/////////////////////////////////////////////////////////////////////////////////////

	dyn_sys_msd_nml_unc(dt_z_unc, Q_in, z, u_in, M_sys, B_sys, K_sys, q_eq);

	/////////////////////////////////////////////////////////////////////////////////////
	// Constrained dynamics:
	/////////////////////////////////////////////////////////////////////////////////////

	// Constraint matrix transpose:
	A_con_tr = nml_mat_transp(A_con);

	// Zeros matrix:
	nml_mat_all_set(zero_m, 0.0); // just to be sure

	/////////////////////////////////////////////////////////////////////////////////////
	// Build mass & constraint matrix (mu):
	/////////////////////////////////////////////////////////////////////////////////////

	// NOTE: nml_mat* mu is (N_q + N_c) x (N_q + N_c)

	r_i_loc = 0;
	for (r_i = 0; r_i < N_q; r_i++) {
		c_i_loc = 0;
		for (c_i = 0; c_i < N_q; c_i++) {
			mu->data[r_i][c_i] = M_sys  ->data[r_i_loc][c_i_loc];
			c_i_loc++;
		}
		c_i_loc = 0;
		for (c_i = N_q; c_i < N_q + N_c; c_i++) {
			mu->data[r_i][c_i] = A_con_tr->data[r_i_loc][c_i_loc];
			c_i_loc++;
		}
		r_i_loc++;
	}

	r_i_loc = 0;
	for (r_i = N_q; r_i < N_q + N_c; r_i++) {
		c_i_loc = 0;
		for (c_i = 0; c_i < N_q; c_i++) {
			mu->data[r_i][c_i] = A_con ->data[r_i_loc][c_i_loc];
			c_i_loc++;
		}
		c_i_loc = 0;
		for (c_i = N_q; c_i < N_q + N_c; c_i++) {
			mu->data[r_i][c_i] = zero_m->data[r_i_loc][c_i_loc];
			c_i_loc++;
		}
		r_i_loc++;
	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Build extended inputs vector:
	/////////////////////////////////////////////////////////////////////////////////////

	nml_mat_all_set(Q_in_ext, 0.0); // just to be sure

	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++)
		Q_in_ext->data[c_i][0] = Q_in->data[c_i][0];

	/////////////////////////////////////////////////////////////////////////////////////
	// Constrained system output - computation:
	/////////////////////////////////////////////////////////////////////////////////////

	// Solve for acceleration & Lagrange multipliers' vector:
	U = nml_mat_cp(mu);
	P = nml_mat_eye(mu->num_rows);

	// was nml_mat_lup_solve_ref(mu_LUP, mu, L, U, P);
	nml_mat_lup_solve_ref(mu, L, U, P, 	&num_permutations);

	// was nml_ls_solve_ref(dt_v_lam, mu_LUP, Q_in_ext, P_Q_in_ext_prod, y_dt_v_lam);
	nml_ls_solve_ref(dt_v_lam, L, U, P, Q_in_ext, P_Q_in_ext_prod, y_dt_v_lam);

	// Constrained output vector:
	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++) {
		dt_z_con->data[c_i][0]       =        v->data[c_i][0];
		dt_z_con->data[c_i + N_q][0] = dt_v_lam->data[c_i][0];
	}

	// ITM console output:
#if USE_ITM_OUT_DYN_SYS_MATR
	static int step_i = 0;
	static int DECIM_DISP_CON = DECIM_DISP_GENERAL;

	if ((step_i % DECIM_DISP_CON) == 0) {
		printf("____________________________\n");
		printf("dyn_sys_msd_nml_constr_lagr [%d]:\n", step_i);
		printf("Q_in_ext:\n");
		for (r_i = 0; r_i < Q_in_ext->num_rows; r_i++) {
			for (c_i = 0; c_i < Q_in_ext->num_cols; c_i++)
				printf("%f\t", Q_in_ext->data[r_i][c_i]);
			printf("\n");
		}

		printf("mu:\n");
		for (r_i = 0; r_i < mu->num_rows; r_i++) {
			for (c_i = 0; c_i < mu->num_cols; c_i++)
				printf("%f\t", mu->data[r_i][c_i]);
			printf("\n");
		}

		printf("\n");
		printf("L:\n");
		for (r_i = 0; r_i < L->num_rows; r_i++) {
			for (c_i = 0; c_i < L->num_cols; c_i++)
				printf("%f\t", L->data[r_i][c_i]);
			printf("\n");
		}

		printf("\n");
		printf("U:\n");
		for (r_i = 0; r_i < U->num_rows; r_i++) {
			for (c_i = 0; c_i < U->num_cols; c_i++)
				printf("%f\t", U->data[r_i][c_i]);
			printf("\n");
		}

		printf("\n");
		printf("P:\n");
		for (r_i = 0; r_i < P->num_rows; r_i++) {
			for (c_i = 0; c_i < P->num_cols; c_i++)
				printf("%f\t", P->data[r_i][c_i]);
			printf("\n");
		}

		printf("dt_v_lam: ");
		for (r_i = 0; r_i < dt_v_lam->num_rows; r_i++) {
			for (c_i = 0; c_i < dt_v_lam->num_cols; c_i++)
				printf("%f\t", dt_v_lam->data[r_i][c_i]);
			printf("\n");
		}
		printf("\n");
	}

	step_i++;
#endif
}

