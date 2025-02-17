/////////////////////////////////////////////////////////////////////////////
//
// _std_c.h
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LL2_STD_C_H_
#define LL2_STD_C_H_

////////////////////////////////////////////////////////////////////////////
// #include files:
////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
// #include <unistd.h>
#include <math.h>
#include <stddef.h>
#include <malloc.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>

////////////////////////////////////////////////////////////////////////////
// General-purpose constants:
////////////////////////////////////////////////////////////////////////////

#define			PI						3.14159265358979
#define			E_BASE					2.71828182845905
#define			INF						1.0e50

////////////////////////////////////////////////////////////////////////////
// Display variables:
////////////////////////////////////////////////////////////////////////////

// 0.5  sec = 100 @ 5 msec sampling
// 0.05 sec =  10 @ 5 msec sampling
#define DECIM_DISP_GENERAL	10

#endif /* LL2_STD_C_H_ */
