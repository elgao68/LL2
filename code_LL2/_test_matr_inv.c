//////////////////////////////////////////////////////////////////////////////
//
//  _test_matr_inv.c
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <_test_matr_inv.h>

void
test_matr_inv() {

	/////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////
	// Test matrix inverse function based on nml_mat_lup_solve_ref() and nml_ls_solve_ref():
	/////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////
	// CASE 1 (5 x 5 matrix):
	/////////////////////////////////////////////////////////////////////////////

	/*
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

	M inverse from Matlab:

	    0.4249   -0.0032    0.5494   -0.3052   -0.1631
	   -0.1433   -0.2148   -0.2298   -0.2372    0.0416
	    0.2881    0.2914    0.8198    0.4089    0.0409
	    0.5995    0.3904    1.1489    0.1551   -0.0835
	    0.4409    0.0517    0.3243   -0.2100   -0.0715
	*/

	/////////////////////////////////////////////////////////////////////////////
	// CASE 2: mu matrix from LL UK equations (5 x 5)
	/////////////////////////////////////////////////////////////////////////////

	double M_data[] = {
	   10.0000,         0,         0,    1.0000,         0,
			 0,   10.0000,         0,         0,    1.0000,
			 0,         0,    1.0000,         0,   -0.1000,
		1.0000,         0,         0,         0,         0,
			 0,    1.0000,   -0.1000,         0,         0};

	nml_mat* M    = nml_mat_from(5, 5, 25, M_data);
	nml_mat* M_sw = nml_mat_new(M->num_rows, M->num_rows);

	/*
	M inverse from Matlab:

	inv_M_con =
			 0         0         0    1.0000         0
			 0    0.0091    0.0909         0    0.9091
			 0    0.0909    0.9091         0   -0.9091
		1.0000         0         0  -10.0000         0
			 0    0.9091   -0.9091         0   -9.0909
	*/

	// Matrix inverse arguments:
	nml_mat* I_col = nml_mat_new(M->num_rows, 1); // equation system's RHS term: single-column placeholder

	nml_mat* inv_M     = nml_mat_new(M->num_rows, M->num_rows); // matrix inverse: initial allocation
	nml_mat* inv_M_sw  = nml_mat_new(M->num_rows, M->num_rows); // matrix inverse (swapped): initial allocation
	nml_mat* inv_M_col = nml_mat_new(M->num_rows, 1); // matrix inverse: single-column placeholder

	nml_mat* L = nml_mat_new(M->num_rows, M->num_rows);
	nml_mat* U = nml_mat_new(M->num_rows, M->num_rows);
	nml_mat* P = nml_mat_new(M->num_rows, M->num_rows);

	nml_mat* P_I_col = nml_mat_new(P->num_rows, M->num_cols); // NOTE: technically second argument should be eye->num_cols
	nml_mat* y_inv_M  = nml_mat_new(U->num_cols, 1); // needed by nml_ls_solve_ref();

	// Counters:
	int r_i, c_i;

	// Swap indices array (5-element):
	int idx_swap[] = {3, 1, 4, 0, 2};

	// Swap initial matrix columns:
	nml_swap_cols_array_ref(M_sw, M, idx_swap);

	// Solve matrix inverse for swapped matrix:
	nml_mat_inv_ref(inv_M_sw, M_sw, L, U, P,
			        I_col, inv_M_col, P_I_col, y_inv_M);

	// Swap inverse matrix rows to obtain desired inverse:
	nml_swap_rows_array_ref(inv_M, inv_M_sw, idx_swap);

	// Display matrix & inverse:
	printf("\n");
	printf("M:\n");
	for (r_i = 0; r_i < M->num_rows; r_i++) {
		for (c_i = 0; c_i < M->num_cols; c_i++)
			printf("%f\t", M->data[r_i][c_i]);
		printf("\n");
	}

	printf("\n");
	printf("M swap:\n");
	for (r_i = 0; r_i < M_sw->num_rows; r_i++) {
		for (c_i = 0; c_i < M_sw->num_cols; c_i++)
			printf("%f\t", M_sw->data[r_i][c_i]);
		printf("\n");
	}

	printf("\n");
	printf("inv M swap:\n");
	for (r_i = 0; r_i < inv_M_sw->num_rows; r_i++) {
		for (c_i = 0; c_i < inv_M_sw->num_cols; c_i++)
			printf("%f\t", inv_M_sw->data[r_i][c_i]);
		printf("\n");
	}

	printf("\n");
	printf("inv M:\n");
	for (r_i = 0; r_i < inv_M->num_rows; r_i++) {
		for (c_i = 0; c_i < inv_M->num_cols; c_i++)
			printf("%f\t", inv_M->data[r_i][c_i]);
		printf("\n");
	}
}
