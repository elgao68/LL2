
/////////////////////////////////////////////////////////////////////////////
//
// linear_systems.h
//
// Created on: 2024.06.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LINEAR_SYSTEMS_H
#define LINEAR_SYSTEMS_H

#include "_std_c.h"

double low_pass_discrete_fwd(double u, double y_prev, double omega, double dt);

double int_hi_pass_discrete_fwd(double u, double y_prev, double omega, double dt);

#endif
