/////////////////////////////////////////////////////////////////////////////
//
// dynamic_systems_nml.h
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LL2_DYNAMIC_SYSTEMS_NML_H_
#define LL2_DYNAMIC_SYSTEMS_NML_H_

#include <_coord_system.h>
#include <nml.h>
#include <nml_util.h>
#include <_std_c.h>
#include <nml_ref_based.h>
#include <ode_solvers_nml.h>

void
dyn_sys_msd_nml_unc(nml_mat* dt_z, nml_mat* Q_in, nml_mat* z, nml_mat* u_in,
		nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, nml_mat* q_eq);

void
dyn_sys_msd_nml_constr_lagr(nml_mat* dt_z, nml_mat* Q_in, nml_mat* z, nml_mat* u_in,
		nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, nml_mat* q_eq, nml_mat* A_con);

#endif /* LL2_DYNAMIC_SYSTEMS_NML_H_ */
