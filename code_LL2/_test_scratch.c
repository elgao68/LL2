/////////////////////////////////////////////////////////////////////////////
//
//  _test_scratch.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <_test_scratch.h>

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// "SCRATCH" SCRIPT - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_scratch() {

	/////////////////////////////////////////////////////////////////////////////
	// Test console:
	/////////////////////////////////////////////////////////////////////////////

	int i = 0;
	for (i = 1; i <= 10; i++) {
		printf("foo[%d]\n", i);
		HAL_Delay(100);
	}
	printf("\n");

	/////////////////////////////////////////////////////////////////////////////
	// Matrix multiplication:
	/////////////////////////////////////////////////////////////////////////////

	double a[] = {
			0.11, 0.12, 0.13,
			0.21, 0.22, 0.23 };
	double b[] = {
			1011, 1012,
			1021, 1022,
			1031, 1032 };

	nml_mat* A = nml_mat_from(2, 3, 6, a);
	nml_mat* B = nml_mat_from(3, 2, 6, b);

	// Multiply matrices
	nml_mat* C = nml_mat_dot(A, B);

	double CC[4];

	CC[0] = C->data[0][0];
	CC[1] = C->data[0][1];

	CC[2] = C->data[1][0];
	CC[3] = C->data[1][1];

	nml_mat* D = nml_mat_sqr(4);

	/////////////////////////////////////////////////////////////////////////////
	// Solve linear system with nml_mat_lup*:
	/////////////////////////////////////////////////////////////////////////////

	double a_data[] = {
		0.18, 0.60, 0.57, 0.96,
		0.41, 0.24, 0.99, 0.58,
		0.14, 0.30, 0.97, 0.66,
		0.51, 0.13, 0.19, 0.85 };

	double b_data[] = { 1.0, 2.0, 3.0, 4.0 };

	nml_mat* A_sys = nml_mat_from(4, 4, 16, a_data);
	nml_mat* b_sys = nml_mat_from(4, 1,  4, b_data);

	nml_mat_lup* A_LUP = nml_mat_lup_solve(A_sys);
	nml_mat*     x     = nml_ls_solve(A_LUP, b_sys);

	double xx[4];

	xx[0] = x->data[0][0];
	xx[1] = x->data[1][0];
	xx[2] = x->data[2][0];
	xx[3] = x->data[3][0];

	nml_mat* E = nml_mat_sqr(4);

	int r_i, c_i;
	for (r_i = 0; r_i < 2; r_i++) {
		printf("\n");
		for (c_i = 0; c_i < 2; c_i++) {
			printf("C[%d][%d] = %f\n", r_i, c_i, C->data[r_i][c_i]);
			HAL_Delay(100);
			// fflush(stdout);
		}
	}

	/////////////////////////////////////////////////////////////////////////////
	// Obtain matrix inverse with nml_mat_lup_solve_ref() and nml_ls_solve_ref():
	/////////////////////////////////////////////////////////////////////////////

	// CASE 1:
	nml_mat* M = nml_mat_new(5, 5);

	M->data[0][0] =  0.1;
	M->data[0][1] = -2.0;
	M->data[0][2] =  3.0;
	M->data[0][3] = -4.0;
	M->data[0][4] =  5.0;

	M->data[1][0] = -6.0;
	M->data[1][1] =  0.7;
	M->data[1][2] = -8.0;
	M->data[1][3] =  9.0;
	M->data[1][4] = -1.0;

	M->data[2][0] =  1.0;
	M->data[2][1] =  2.0;
	M->data[2][2] =  0.5;
	M->data[2][3] =  1.0;
	M->data[2][4] = -2.0;

	M->data[3][0] =  3.0;
	M->data[3][1] = -4.0;
	M->data[3][2] =  5.0;
	M->data[3][3] = -6.0;
	M->data[3][4] = 0.7;

	M->data[4][0] = -8.0;
	M->data[4][1] =  9.0;
	M->data[4][2] =  0.3;
	M->data[4][3] =  4.0;
	M->data[4][4] =  5.0;

	/*
	M inverse from Matlab:

	    0.4249   -0.0032    0.5494   -0.3052   -0.1631
	   -0.1433   -0.2148   -0.2298   -0.2372    0.0416
	    0.2881    0.2914    0.8198    0.4089    0.0409
	    0.5995    0.3904    1.1489    0.1551   -0.0835
	    0.4409    0.0517    0.3243   -0.2100   -0.0715
	*/

	// CASE 2: M_con matrix from LL UK equations:
	/*
	double M_data[] = {
	         0,         0,         0,
	         0,         0,    0.1000,
	         0,         0,    1.0000,
	    1.0000,         0,         0,
	         0,    1.0000,   -0.1000};

	nml_mat* M = nml_mat_from(5, 3, 15, M_data);

	M inverse from Matlab:

	inv_M_con =
	         0         0         0    1.0000         0
	   -0.0000    0.0099    0.0990         0    1.0000
	   -0.0000    0.0990    0.9901         0   -0.0000
	*/

	// Obtain matrix inverse:
	nml_mat* I = nml_mat_new(M->num_rows, M->num_rows);
	nml_mat_eye_ref(I); // equation system's RHS term
	nml_mat* I_col = nml_mat_new(M->num_rows, 1); // equation system's RHS term: single-column placeholder

	nml_mat* inv_M     = nml_mat_new(M->num_rows, M->num_rows); // matrix inverse: initial allocation
	nml_mat* inv_M_col = nml_mat_new(M->num_rows, 1); // matrix inverse: single-column placeholder

	nml_mat* L = nml_mat_new(M->num_rows, M->num_rows);
	nml_mat* U = nml_mat_new(M->num_rows, M->num_rows);
	nml_mat* P = nml_mat_new(M->num_rows, M->num_rows);

	nml_mat* P_I_col = nml_mat_new(P->num_rows, I->num_cols); // NOTE: I is actually not used elsewhere
	nml_mat* y_inv_M  = nml_mat_new(U->num_cols, 1); // needed by nml_ls_solve_ref();

	unsigned int num_permutations;

	// Solve matrix inverse one column at a time:
	for (c_i = 0; c_i < inv_M->num_cols; c_i++) {
		nml_mat_cp_ref(U, M);
		nml_mat_eye_ref(P);

		// LUP decomposition:
		nml_mat_lup_solve_ref(M, L, U, P, &num_permutations);

		// Generate column of RHS (identity matrix):
		nml_mat_all_set(I_col, 0.0);
		I_col->data[c_i][0] = 1.0;

		nml_mat_all_set(inv_M_col, 0.0);
		nml_ls_solve_ref(inv_M_col, L, U, P, I_col, P_I_col, y_inv_M);

		// Assign solution to matrix inverse column:
		for (r_i = 0; r_i < inv_M->num_rows; r_i++) {
			inv_M->data[r_i][c_i] = inv_M_col->data[r_i][0];
		}
	}

	// Display matrix inverse:
	printf("\n");
	printf("inv M:\n");
	for (r_i = 0; r_i < inv_M->num_rows; r_i++) {
		for (c_i = 0; c_i < inv_M->num_cols; c_i++)
			printf("%f\t", inv_M->data[r_i][c_i]);
		printf("\n");
	}
}
