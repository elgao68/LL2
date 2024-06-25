/////////////////////////////////////////////////////////////////////////////
//
// linear_systems.c
//
// Created on: 2024.06.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include "linear_systems.h"

double
low_pass_discrete_fwd(double u, double y_prev, double omega, double dt) {
	double A = omega*dt/(1 + omega*dt);
	double B =        1/(1 + omega*dt);

	return A*u + B*y_prev;
}

double
int_hi_pass_discrete_fwd(double u, double y_prev, double omega, double dt) {
	double A = dt/(1 + omega*dt);
	double B =  1/(1 + omega*dt);

	return A*u + B*y_prev;
}
