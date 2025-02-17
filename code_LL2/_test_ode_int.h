/////////////////////////////////////////////////////////////////////////////
//
//  _test_ode_int.h
//
// Created on: 2024.03.20
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CODE_LL2__TEST_ODE_INT_H_
#define CODE_LL2__TEST_ODE_INT_H_

#include <_std_c.h>
#include <control_settings_ll2.h>
#include <motor_algo_ll2.h>
#include <nml.h>
#include <nml_util.h>
#include <traj_ctrl_params_nml.h>
#include "timer.h"

// Dynamic system mode: unconstrained / constrained:
#define ADMITT_MODEL_CONSTR_ON		1
#define ADMITT_MODEL_CONSTR_OFF		0

void test_ode_int();

#endif /* CODE_LL2__TEST_ODE_INT_H_ */
