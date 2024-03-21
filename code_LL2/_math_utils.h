////////////////////////////////////////////////////////////////////////////                                                                        
// utils.h                                                              
// Description:		header file,
//					assorted utility functions                                
//                                                    
// Gabriel Aguirre-Ollinger
// Start date: 02.21.2005
// Modifications record:
//		03.21.2024: NOTE functions need to be tested on a cas-by-case basis
////////////////////////////////////////////////////////////////////////////

#ifndef UTILS_H
#define UTILS_H

#define NR_END				1
#define FREE_ARG			char*
#define	TINY				1e-20
#define INIT_STEP_ODE_SOLVE	2

#define	ORD_2		2

#include <_std_c.h>

////////////////////////////////////////////////////////////////////////////
// Parameter struct to be passed along with a pointer to function (ODE solvers):
////////////////////////////////////////////////////////////////////////////

#define	NUM_DBL_PARAMS		10
#define	NUM_INT_PARAMS		10
	
typedef struct param_struct {
	double		p_dbl[NUM_DBL_PARAMS];
	int			p_int[NUM_INT_PARAMS];
} param_struct;

////////////////////////////////////////////////////////////////////////////
// General-purpose functions:
////////////////////////////////////////////////////////////////////////////

int	round_dbl(double value);
double	norm2(double* vect, int dim);
double	int_trapezoidal(double* x, double y_init, double n, double T_INC);
double	normald_angle(double ang);
int		sgn(double val);
double	std_dev_moving_static(double x_in, double* mean_x, const int num_samples);
double	cubic_interpolation(double* y, double u);
double  vel_cubic_interpolation(double* y, double u, double dt_u);
double	normal_pdf(double x, double mean, double stdev);
double	t_integrator_ode(double x, param_struct params);
int 	factorial(int n);

////////////////////////////////////////////////////////////////////////
// Numerical integration methods:
////////////////////////////////////////////////////////////////////////

void	solve_ode_runge_kutta_sys1_ord4(double* x, double T, double (*ode_func)(double, param_struct), param_struct params);
void	solve_ode_runge_kutta_sys2_ord4(double* x, double *dt_x, double T, double (*ode_func)(double, double, param_struct), 
                                      param_struct params);
void	solve_ode_pred_corr_sys2_ord2_static(double* x_0, double* dt_x_0, double T, double (*f)(double, double, param_struct), 
                                             param_struct par);

////////////////////////////////////////////////////////////////////////
// Matrix operations taken from "Numerical recipes in C", Ch. 2.3
////////////////////////////////////////////////////////////////////////

double* vector(int nl, int nh);
int*	ivector(int nl, int nh);
void	free_vector(double* v, int nl, int nh);

void	lu_decomp(double **a, int n, int *indx, double *d);
void	lu_backsub(double **a, int n, int *indx, double* b);
void	matr_inv_lu(double **a, int n);
void	gauss_jordan_pivot(double **a, int n, double **b, int m);

#endif
