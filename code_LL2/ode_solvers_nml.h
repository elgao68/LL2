/////////////////////////////////////////////////////////////////////////////
//
// ode_solvers_nml.h
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LL2_ODE_SOLVERS_NML_H_
#define LL2_ODE_SOLVERS_NML_H_

#include <nml.h>
#include <nml_util.h>
#include <_ode_solver_params.h>
#include <_coord_system.h>
#include <nml_ref_based.h>
#include <_std_c.h>

void
solve_ode_sys_rectang_nml(nml_mat* y_f, nml_mat* y_o, double T,
		void (*ode_func)(nml_mat*, nml_mat*, ode_param_struct), // ensure that every ode_ function declaration is consistent with this structure
		ode_param_struct ode_params);

/*
void
solve_ode_sys_rkutta_ord4_nml(nml_mat* y_f, nml_mat* y_o, double T,
		void (*ode_func)(nml_mat*, nml_mat*, ode_param_struct), // ensure that every ode_ function declaration is consistent with this structure
		ode_param_struct ode_params);
*/

#endif /* LL2_ODE_SOLVERS_NML_H_ */
