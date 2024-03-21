/////////////////////////////////////////////////////////////////////////////
//
// _nml_ref_based.c
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <nml_ref_based.h>

#define CANNOT_MULT \
  "Cannot multiply matrices: number of columns of the first one is different than the number of rows of the second one.\n" \

#define CANNOT_MULT_ELEM \
  "Cannot multiply matrices element-wise: number of rows and number of columns should match.\n" \

#define CANNOT_LU_MATRIX_SQUARE \
	"Cannot LU. Matrix (%d, %d) needs to be square.\n" \

#define CANNOT_LU_MATRIX_DEGENERATE \
  "Cannot LU. Matrix is degenerate or almost degenerate.\n" \

#define CANNOT_SOLVE_LIN_SYS_INVALID_B \
	"Cannot solve system. Wrong-sized matrix.\n" \

void nml_mat_eye_ref(nml_mat *A) {
	nml_mat_all_set(A, 0.0);

	for (int r_i = 0; r_i < A->num_rows; r_i++)
		A->data[r_i][r_i] = 1.0;
}

void nml_mat_dot_ref(nml_mat *m_out, nml_mat *m1, nml_mat *m2) {
	if (!(m1->num_cols == m2->num_rows))
		if (DEBUG_TRUE) {
			nml_log(stderr, __FILE__, __LINE__, CANNOT_MULT);
		}

	nml_mat_all_set(m_out, 0.0);

	int i, j, k;
	for(i = 0; i < m_out->num_rows; i++)
		for(j = 0; j < m_out->num_cols; j++)
			for(k = 0; k < m1->num_cols; k++)
				m_out->data[i][j] += m1->data[i][k] * m2->data[k][j];
}

void nml_mat_mult_elemwise_ref(nml_mat *m_out, nml_mat *m1, nml_mat *m2) {
	if ((m1->num_rows != m2->num_rows) || (m1->num_cols != m2->num_cols))
		if (DEBUG_TRUE) {
			nml_log(stderr, __FILE__, __LINE__, CANNOT_MULT_ELEM);
		}

	int i,j;
	for(i = 0; i < m1->num_rows; i++)
		for(j = 0; j < m1->num_cols; j++)
			m_out->data[i][j] = m1->data[i][j] * m2->data[i][j];
}

void nml_mat_cp_ref(nml_mat *m_out, nml_mat *m) {
	// Safety catch - assumption is that m_out dimensions were correctly specified with nml_mat_new():
	m_out->num_rows = m->num_rows;
	m_out->num_cols = m->num_cols;

	int i,j;
	for(i = 0; i < m->num_rows; i++)
		for(j = 0; j < m->num_cols; j++)
			m_out->data[i][j] = m->data[i][j];
}

/*
LUP solver - old definition:
void nml_mat_lup_solve_ref(nml_mat_lup *r, nml_mat *m, nml_mat *L, nml_mat *U, nml_mat *P),
returned return nml_mat_lup_new_ref(r, L, U, P, num_permutations);
*/

// LUP solver - new definition - uses no nml_mat_lup* struct:
void nml_mat_lup_solve_ref(nml_mat *m, nml_mat *L, nml_mat *U, nml_mat *P, unsigned int* num_permutations) {
  if (!m->is_square) {
	nml_log(stderr, __FILE__, __LINE__, CANNOT_LU_MATRIX_SQUARE);
  }

  int j,i, pivot;
  double mult;

  for(j = 0; j < U->num_cols; j++) {
    // Retrieves the row with the biggest element for column (j)
    pivot = _nml_mat_absmaxr(U, j);
    if (fabs(U->data[pivot][j]) < NML_MIN_COEF) {
  	  nml_log(stderr, __FILE__, __LINE__, CANNOT_LU_MATRIX_DEGENERATE);
    }

    if (pivot!=j) {
      // Pivots LU and P accordingly to the rule
      nml_mat_row_swap_r(U, j, pivot);
      nml_mat_row_swap_r(L, j, pivot);
      nml_mat_row_swap_r(P, j, pivot);
      // Keep the number of permutations to easily calculate the
      // determinant sign afterwards
      num_permutations++;
    }

    for(i = j + 1; i < U->num_rows; i++) {
      mult = U->data[i][j] / U->data[j][j];
      // Building the U upper rows
      nml_mat_row_addrow_r(U, i, j, -mult);
      // Store the multiplier in L
      L->data[i][j] = mult;
    }
  }
  nml_mat_diag_set(L, 1.0);
}

void nml_mat_lup_new_ref(nml_mat_lup *r, nml_mat *L, nml_mat *U, nml_mat *P, unsigned int num_permutations) {
  r->L = L;
  r->U = U;
  r->P = P;
  r->num_permutations = num_permutations;
}

// A[n][n] is a square matrix
// m contains matrices L, U, P for A[n][n] so that P*A = L*U
//
// The linear system is:
// A*x=b  =>  P*A*x = P*b  =>  L*U*x = P*b  =>
// (where b is a matrix[n][1], and x is a matrix[n][1])
//
// if y = U*x , we solve two systems:
//    L * y = P b (forward substitution)
//    U * x = y (backward substitution)
// We obtain and return x:

// Old implementation:
// void nml_ls_solve_ref(nml_mat *x_soln, nml_mat_lup *lu, nml_mat* b, nml_mat *Pb, nml_mat *y)

// New implementation - no nml_mat_lup* struct:
void nml_ls_solve_ref(nml_mat *x_soln, nml_mat *L, nml_mat *U, nml_mat *P, nml_mat* b, nml_mat *Pb, nml_mat *y) {
	if (U->num_rows != b->num_rows || b->num_cols != 1) {
		nml_log(stderr, __FILE__, __LINE__, CANNOT_SOLVE_LIN_SYS_INVALID_B);
	}
	nml_mat_dot_ref(Pb, P, b);

	// We solve L*y = P*b using forward substitution
	nml_ls_solvefwd_ref(y, L, Pb);

	// We solve U*x_soln=y
	nml_ls_solvebck_ref(x_soln, U, y);
}

// Forward substitution algorithm
// Solves the linear system L * x = b
//
// L is lower triangular matrix of size NxN
// B is column matrix of size Nx1
// x is the solution column matrix of size Nx1
//
// Note: In case L is not a lower triangular matrix, the algorithm will try to
// select only the lower triangular part of the matrix L and solve the system
// with it.
//
// Note: In case any of the diagonal elements (L[i][i]) are 0 the system cannot
// be solved
//
// Note: This function is usually used with an L matrix from a LU decomposition
void nml_ls_solvefwd_ref(nml_mat* x, nml_mat *L, nml_mat *b) {
	int i,j;
	double tmp;
	for(i = 0; i < L->num_cols; i++) {
		tmp = b->data[i][0];
		for(j = 0; j < i ; j++) {
			tmp -= L->data[i][j] * x->data[j][0];
		}
		x->data[i][0] = tmp / L->data[i][i];
	}
}

// Back substitution algorithm
// Solves the linear system U *x = b
//
// U is an upper triangular matrix of size NxN
// B is a column matrix of size Nx1
// x is the solution column matrix of size Nx1
//
// Note in case U is not an upper triangular matrix, the algorithm will try to
// select only the upper triangular part of the matrix U and solve the system
// with it
//
// Note: In case any of the diagonal elements (U[i][i]) are 0 the system cannot
// be solved
void nml_ls_solvebck_ref(nml_mat *x, nml_mat *U, nml_mat *b) {
	int i = U->num_cols, j;
	double tmp;
	while(i-->0) {
		tmp = b->data[i][0];
		for(j = i; j < U->num_cols; j++) {
			tmp -= U->data[i][j] * x->data[j][0];
		}
		x->data[i][0] = tmp / U->data[i][i];
	}
}

// Matrix exponential:
void nml_exp_matrix_ref(nml_mat *exp_A, nml_mat *A, nml_mat *pow_A, nml_mat* term_buff, double T, int N_STEPS) {

	// WARNING: no dimensional check is performed

	nml_mat_eye_ref(exp_A); // initialize sum to identity matrix
	nml_mat_eye_ref(pow_A); // initialize power of A to identity matrix (A^0)

	for (int i = 1; i < N_STEPS; i++) {
		// Compute power of A (A^i):
		nml_mat_dot_ref(term_buff, pow_A, A);
		nml_mat_cp_ref(pow_A, term_buff);

		// Compute new term in sum, noting that term_buff == pow_A:
		nml_mat_smult_r(term_buff, pow(T, i)/factorial(i));
		nml_mat_add_r(exp_A, term_buff);
	}
}

