/////////////////////////////////////////////////////////////////////////////
//
// _coord_system.h
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////


#ifndef LL2_COORD_SYSTEM_H_
#define LL2_COORD_SYSTEM_H_

#define N_COORD_2D   	2 // coordinate system dimension (planar)
#define N_COORD_EXT  	3 // extended coordinate system dimension (includes phase)
#define N_CONSTR_TRAJ	2 // trajectory constraints dimension

#define N_POS_VEL_2D	4

// Indices of extended coordinates and their first derivatives ("z" state):
#define IDX_X        0
#define IDX_Y        1
#define IDX_PHI      2
#define IDX_DT_X     3
#define IDX_DT_Y     4
#define IDX_DT_PHI   5

#endif /* LL2_COORD_SYSTEM_H_ */
