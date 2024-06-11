/////////////////////////////////////////////////////////////////////////////
//
// dynamic_systems_nml.c
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <dynamic_systems_nml.h>

#define USE_ITM_OUT_DYN_SYS_UNC		0
#define USE_ITM_OUT_DYN_SYS_CON		0

void
dyn_sys_msd_nml_unc(nml_mat* dt_z_unc, nml_mat* Q_in, nml_mat* z, nml_mat* u_in,
		nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, nml_mat* q_eq) { // A_con

	size_t N_z = z->num_rows;
	size_t N_q = N_z/2;

	// Counters:
	int c_i, r_i;

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

	// Inertia matrix inverse:
	for (c_i = 0; c_i < M_sys->num_cols; c_i++)
		inv_M_sys->data[c_i][c_i] = 1.0/M_sys->data[c_i][c_i];

	// ITM console output:
#if USE_ITM_OUT_DYN_SYS_UNC
	static int step_i = 0;
	int DECIM_DISP_UNC = DECIM_DISP_GENERAL;

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

		printf("inv_M_sys:\n");
		for (r_i = 0; r_i < inv_M_sys->num_rows; r_i++) {
			for (c_i = 0; c_i < inv_M_sys->num_cols; c_i++)
				printf("%f\t", inv_M_sys->data[r_i][c_i]);
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

	static int step_i = 0;
	int DECIM_DISP_CON = DECIM_DISP_GENERAL;

    /////////////////////////////////////////////////////////////////////////////////////
    // Declare static matrices:
    /////////////////////////////////////////////////////////////////////////////////////

	// (1) Kinematics arrays:
	static nml_mat* v;
	static nml_mat* dt_z_unc;

	// (2) Constraint matrix and auxiliary matrices:
	static nml_mat* A_con_tr;
	static nml_mat* zero_m;

	// (3) Extended inertia matrix and inverse-related matrices:
	static nml_mat* mu;
	static nml_mat* mu_sw; // inertia matrix, swapped
	static nml_mat* inv_mu; // matrix inverse
	static nml_mat* inv_mu_sw; // matrix inverse (swapped)
	static nml_mat* inv_mu_col; // matrix inverse: single-column placeholder

	// (4) LU decomposition matrices:
	static nml_mat* L;
	static nml_mat* U;
	static nml_mat* P;

	// (5) Matrix inversion: auxiliary matrices
	static nml_mat* I_col; // equation system's RHS term: single-column placeholder
	static nml_mat* P_I_col;
	static nml_mat* y_inv_mu;

	// (6) State-space dynamic system: input and first derivatives arrays:
	static nml_mat *dt_v_lam;
	static nml_mat* Q_in_ext;

	/////////////////////////////////////////////////////////////////////////////////////
	// Swap indices array (CRITICAL FOR MATRIX INVERSE USING LU):
	/////////////////////////////////////////////////////////////////////////////////////

	int IDX_SWAP[] = {3, 1, 4, 0, 2};

	/////////////////////////////////////////////////////////////////////////////////////
    // Initialize matrices:
    /////////////////////////////////////////////////////////////////////////////////////

	static int init_nml = 1;

	if (init_nml) {
		// (1) Kinematics arrays:
		v        = nml_mat_new(N_q, 1);
		dt_z_unc = nml_mat_new(N_z, 1);

		// (2) Constraint matrix and auxiliary matrices:
		A_con_tr = nml_mat_new(N_q, N_c);
		zero_m   = nml_mat_new(N_c, N_c);

		// (3) Extended inertia matrix and inverse-related matrices:
		mu         = nml_mat_new(N_q + N_c, N_q + N_c);
		mu_sw      = nml_mat_new(mu->num_rows, mu->num_rows); // inertia matrix, swapped

		inv_mu     = nml_mat_new(mu->num_rows, mu->num_rows); // matrix inverse: initial allocation
		inv_mu_sw  = nml_mat_new(mu->num_rows, mu->num_rows); // matrix inverse (swapped): initial allocation
		inv_mu_col = nml_mat_new(mu->num_rows, 1); // matrix inverse: single-column placeholder

		// (4) LU decomposition matrices:
		L  = nml_mat_new(mu->num_rows, mu->num_rows);
		U  = nml_mat_new(mu->num_rows, mu->num_rows);
		P  = nml_mat_new(mu->num_rows, mu->num_rows);

		// (5) Matrix inversion: auxiliary matrices
		I_col   = nml_mat_new(mu->num_rows, 1); // equation system's RHS term: single-column placeholder
		P_I_col = nml_mat_new(P->num_rows, mu->num_cols); // NOTE: technically second argument should be eye->num_cols
		y_inv_mu = nml_mat_new(U->num_cols, 1); // needed by nml_ls_solve_ref();

		// (6) State-space dynamic system: input and first derivatives arrays:
		dt_v_lam = nml_mat_new(N_q + N_c, 1);
		Q_in_ext = nml_mat_new(N_q + N_c, 1);

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
	nml_mat_transp_ref(A_con_tr, A_con);

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

	// ITM console output:
	#if USE_ITM_OUT_DYN_SYS_CON
		// if ((step_i % DECIM_DISP_CON) == 0) {
			printf("____________________________\n");
			printf("dyn_sys_msd_nml_constr_lagr [%d]:\n", step_i);
			printf("\n");
			printf("M_sys:\n");
			for (r_i = 0; r_i < M_sys->num_rows; r_i++) {
				for (c_i = 0; c_i < M_sys->num_cols; c_i++)
					printf("%f\t", M_sys->data[r_i][c_i]);
				printf("\n");
			}

			printf("A_con:\n");
			for (r_i = 0; r_i < A_con->num_rows; r_i++) {
				for (c_i = 0; c_i < A_con->num_cols; c_i++)
					printf("%f\t", A_con->data[r_i][c_i]);
				printf("\n");
			}

			/*
			printf("A_con_tr:\n");
			for (r_i = 0; r_i < A_con_tr->num_rows; r_i++) {
				for (c_i = 0; c_i < A_con_tr->num_cols; c_i++)
					printf("%f\t", A_con_tr->data[r_i][c_i]);
				printf("\n");
			}
			*/

			printf("\n");
			printf("mu:\n");
			for (r_i = 0; r_i < mu->num_rows; r_i++) {
				for (c_i = 0; c_i < mu->num_cols; c_i++)
					printf("%f\t", mu->data[r_i][c_i]);
				printf("\n");
			}

			printf("\n");
			printf("inv mu:\n");
			for (r_i = 0; r_i < inv_mu->num_rows; r_i++) {
				for (c_i = 0; c_i < inv_mu->num_cols; c_i++)
					printf("%f\t", inv_mu->data[r_i][c_i]);
				printf("\n");
			}

			/*
			printf("\n");
			printf("U initial:\n");
			for (r_i = 0; r_i < U->num_rows; r_i++) {
				for (c_i = 0; c_i < U->num_cols; c_i++)
					printf("%f\t", U->data[r_i][c_i]);
				printf("\n");
			}

			printf("\n");
			printf("P initial:\n");
			for (r_i = 0; r_i < P->num_rows; r_i++) {
				for (c_i = 0; c_i < P->num_cols; c_i++)
					printf("%f\t", P->data[r_i][c_i]);
				printf("\n");
			}
			*/

		// } // if ((step_i % DECIM_DISP_CON)
	#endif

	/////////////////////////////////////////////////////////////////////////////////////
	// Invert extended inertia matrix:
	/////////////////////////////////////////////////////////////////////////////////////

	// Swap inertia matrix columns:
	nml_swap_cols_array_ref(mu_sw, mu, IDX_SWAP);

	// Solve matrix inverse for swapped matrix:
	nml_mat_inv_ref(inv_mu_sw, mu_sw, L, U, P,
			I_col, inv_mu_col, P_I_col, y_inv_mu);

	// Swap inverse matrix rows to obtain desired inverse:
	nml_swap_rows_array_ref(inv_mu, inv_mu_sw, IDX_SWAP);

	/////////////////////////////////////////////////////////////////////////////////////
	// Constrained system output:
	/////////////////////////////////////////////////////////////////////////////////////

	nml_mat_dot_ref(dt_v_lam, inv_mu, Q_in_ext);

	// Constrained output vector:
	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++) {
		dt_z_con->data[c_i][0]       =        v->data[c_i][0];
		dt_z_con->data[c_i + N_q][0] = dt_v_lam->data[c_i][0];
	}

	// ITM console output:
	#if USE_ITM_OUT_DYN_SYS_CON
		// if ((step_i % DECIM_DISP_CON) == 0) {
			printf("----------------------------\n");
			printf("dyn_sys_msd_nml_constr_lagr [%d]:\n", step_i);

			printf("\n");
			printf("Q_in_ext = [");
			for (r_i = 0; r_i < Q_in_ext->num_rows; r_i++)
				printf("%f\t", Q_in_ext->data[r_i][0]);
			printf("]\n");

			/*
			printf("\n");
			printf("L final:\n");
			for (r_i = 0; r_i < L->num_rows; r_i++) {
				for (c_i = 0; c_i < L->num_cols; c_i++)
					printf("%f\t", L->data[r_i][c_i]);
				printf("\n");
			}

			printf("\n");
			printf("U final:\n");
			for (r_i = 0; r_i < U->num_rows; r_i++) {
				for (c_i = 0; c_i < U->num_cols; c_i++)
					printf("%f\t", U->data[r_i][c_i]);
				printf("\n");
			}

			printf("\n");
			printf("P final:\n");
			for (r_i = 0; r_i < P->num_rows; r_i++) {
				for (c_i = 0; c_i < P->num_cols; c_i++)
					printf("%f\t", P->data[r_i][c_i]);
				printf("\n");
			}
			*/

			printf("\n");
			printf("dt_v_lam = [");
			for (r_i = 0; r_i < dt_v_lam->num_rows; r_i++)
				printf("%f\t", dt_v_lam->data[r_i][0]);
			printf("]\n");

			printf("\n");
		// } // end if ((step_i % DECIM_DISP_CON)
	#endif

	step_i++;
}

