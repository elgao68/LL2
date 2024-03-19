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

void nml_mat_dot_ref(nml_mat *m_out, nml_mat *m1, nml_mat *m2);

void nml_mat_mult_elemwise_ref(nml_mat *m_out, nml_mat *m1, nml_mat *m2);

void nml_mat_cp_ref(nml_mat *m_out, nml_mat *m);

// void nml_mat_lup_solve_ref(nml_mat_lup *r, nml_mat *m, nml_mat *L, nml_mat *U, nml_mat *P);
void nml_mat_lup_solve_ref(nml_mat *m, nml_mat *L, nml_mat *U, nml_mat *P, unsigned int* num_permutations);

void nml_mat_lup_new_ref(nml_mat_lup *r, nml_mat *L, nml_mat *U, nml_mat *P, unsigned int num_permutations);

// void nml_ls_solve_ref(nml_mat *x_soln, nml_mat_lup *lu, nml_mat* b, nml_mat *Pb, nml_mat *y);
void nml_ls_solve_ref(nml_mat *x_soln, nml_mat *L, nml_mat *U, nml_mat *P, nml_mat* b, nml_mat *Pb, nml_mat *y);

void nml_ls_solvefwd_ref(nml_mat* x, nml_mat *L, nml_mat *b);

void nml_ls_solvebck_ref(nml_mat *x, nml_mat *U, nml_mat *b);

#endif /* CODE_LL2_NML_REF_BASED_H_ */
