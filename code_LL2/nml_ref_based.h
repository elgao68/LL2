/////////////////////////////////////////////////////////////////////////////
//
// _nml_ref_based.h
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
// NOTES:
// This is an extension of the NML matrix operations library by Andrei N. Ciobanu.
// The functions defined here avoid dynamic memory allocation calls (calloc() within nml_mat_new()),
// thereby avoiding allocation fails when running the code on a microcontroller.
// Any matrices needed by a particular function are passed by reference instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2_NML_REF_BASED_H_
#define CODE_LL2_NML_REF_BASED_H_

#include <nml.h>
#include <math.h>
#include <_math_utils.h>

void nml_mat_eye_ref(nml_mat *A);

void nml_mat_transp_ref(nml_mat *r, nml_mat *m);

void nml_mat_dot_ref(nml_mat *m_out, nml_mat *m1, nml_mat *m2);

void nml_mat_mult_elemwise_ref(nml_mat *m_out, nml_mat *m1, nml_mat *m2);

void nml_mat_cp_ref(nml_mat *m_out, nml_mat *m);

void nml_mat_lup_solve_ref(nml_mat *m, nml_mat *L, nml_mat *U, nml_mat *P, unsigned int* num_permutations);

void nml_ls_solve_ref(nml_mat *x_soln, nml_mat *L, nml_mat *U, nml_mat *P, nml_mat* b, nml_mat *Pb, nml_mat *y);

void nml_ls_solvefwd_ref(nml_mat* x, nml_mat *L, nml_mat *b);
void nml_ls_solvebck_ref(nml_mat *x, nml_mat *U, nml_mat *b);

// Matrix exponential:
void nml_exp_matrix_ref(nml_mat *exp_A, nml_mat *A, nml_mat *pow_A, nml_mat* term_buff, double T, int N_STEPS);

// Swap operations:
void nml_swap_cols_array_ref(nml_mat *M_sw, nml_mat *M, int idx_swap[]);
void nml_swap_rows_array_ref(nml_mat *M_sw, nml_mat *M, int idx_swap[]);

// Matrix inverse:
void nml_mat_inv_ref(nml_mat *inv_M, nml_mat *M, nml_mat *L, nml_mat *U, nml_mat *P,
		             nml_mat* I_col, nml_mat* inv_M_col, nml_mat* P_I_col, nml_mat* y_inv_M);

#endif /* CODE_LL2_NML_REF_BASED_H_ */
