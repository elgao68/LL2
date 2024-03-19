/////////////////////////////////////////////////////////////////////////////
//
// _ode_solvers_params.h
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2__ODE_SOLVER_PARAMS_H_
#define CODE_LL2__ODE_SOLVER_PARAMS_H_

#define	N_PARAMS_ODE_DBL 10
#define N_PARAMS_ODE_NML 10

#include <nml.h>

typedef struct ode_param_struct {
	int         DIM;
	int			USE_ODE_CONSTR; // ODE constraint option (0:OFF, 1:ON)
	double		par_dbl[N_PARAMS_ODE_DBL];
	nml_mat*	par_nml[N_PARAMS_ODE_NML];
} ode_param_struct;

#endif /* CODE_LL2__ODE_SOLVER_PARAMS_H_ */
