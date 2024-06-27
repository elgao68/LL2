#ifndef CONTROL_SETTINGS_LL2_
#define CONTROL_SETTINGS_LL2_

///////////////////////////////////////////////////////////////////////////////
// Motor torque activation (CRITICAL):
///////////////////////////////////////////////////////////////////////////////

#define MOTOR_TORQUE_ACTIVE_CALIB		0
#define MOTOR_TORQUE_ACTIVE_JOG			0
#define MOTOR_TORQUE_ACTIVE_EXERCISE	0

///////////////////////////////////////////////////////////////////////////////
// End-effector trajectory parameters - DEFAULT:
///////////////////////////////////////////////////////////////////////////////

#define CYCLE_PERIOD_DEF	3.0
#define EXP_BLEND_TIME		9.0
#define SEMIAXIS_X_DEF		0.15 // 0.12
#define SEMIAXIS_Y_DEF		0.075
#define ROT_ANGLE_DEF		0.0 // (-PI/6)
#define CYCLE_DIR_DEF		1
#define PHI_INIT            (3*PI/2)

#define TEST_CALIB_RUN		0
#define V_CALIB				0.02 // 0.08
#define ALPHA_CALIB         0.1

///////////////////////////////////////////////////////////////////////////////
// Admittance control parameters - DEFAULT:
///////////////////////////////////////////////////////////////////////////////

#define INERTIA_XY_DEF		1.0 // CRITICAL
#define INERTIA_PHI_DEF		0.5 // CRITICAL
#define DAMPING_DEF			0.0
#define STIFFNESS_DEF		0.0

#define F_TANG_DEF	   	   15.0

///////////////////////////////////////////////////////////////////////////////
// Dynamic response parameters - DEFAULT:
///////////////////////////////////////////////////////////////////////////////

#define DAMP_RATIO_DEF	0.2
#define OMEGA_N_DEF		(2*PI*1.0) // natural frequency

///////////////////////////////////////////////////////////////////////////////
// Scaling factors:
///////////////////////////////////////////////////////////////////////////////

#define SCALE_FB	    800.0
#define SCALE_FF		  0.4
#define SCALE_GCOMP	      1.2
#define SCALE_F_END_MEAS  1.0

#define SCALE_FB_CALIB  100.0

///////////////////////////////////////////////////////////////////////////////
//  Feedback control:
///////////////////////////////////////////////////////////////////////////////

// Feedback gains (xv = [x dt_x y dt_y]):
/*
static double K_LQ_XV_DEF[] = {
   10.2006,    0.6046,         0,         0,
		 0,         0,   10.2007,    0.0839
};
*/

static double K_LQ_XV_DEF[] = {
   11.0,    4.0,         0,         0,
	  0,      0,      11.0,       4.0
};

static double K_INT_ERR_POS = 0.0;

///////////////////////////////////////////////////////////////////////////////
// Feedforward control:
///////////////////////////////////////////////////////////////////////////////

// #define F_END_FF_MAX 40.0

// FF 'transfer function':
static double C_FF_DC_DEF[] = {
	749.2168,
	794.0586
};

///////////////////////////////////////////////////////////////////////////////
// Gravity compensation:
///////////////////////////////////////////////////////////////////////////////

// Gravity compensation force (F_g_comp):
static double F_G_COMP_DEF = 15.0;

///////////////////////////////////////////////////////////////////////////////
// Exercise modes:
///////////////////////////////////////////////////////////////////////////////

#define LEN_STR_MAX   35
#define LEN_EXERC_MODE_LIST 5

typedef enum {
	ImpedanceCtrl         = 1,
	PassiveTrajectoryCtrl = 2,
	AdmittanceCtrl        = 3,
	ActiveTrajectoryCtrl  = 4,
} exercise_mode_t;

// Exercise mode strings (must match enumeration):
static char EXERC_MODE_STR[LEN_EXERC_MODE_LIST][LEN_STR_MAX] = {
	"",
	"ImpedanceCtrl",
	"PassiveTrajectoryCtrl",
	"AdmittanceCtrl",
	"ActiveTrajectoryCtrl"
};

///////////////////////////////////////////////////////////////////////////////
// Trajectory types:
///////////////////////////////////////////////////////////////////////////////

#define LEN_TRAJ_TYPE_LIST 4

typedef enum {
	NullTraj       = 0,
	EllipticalTraj = 1,
	LinearTraj     = 2,
	IsometricTraj  = 3
} traj_type_t;

// Exercise mode strings (must match enumeration):
static char TRAJ_TYPE_STR[LEN_TRAJ_TYPE_LIST][LEN_STR_MAX] = {
	"NULL TRAJ",
	"ELLIPTICAL TRAJ",
	"LINEAR TRAJ",
	"ISOMETRIC TRAJ"
};

///////////////////////////////////////////////////////////////////////////////
// Calibration states:
///////////////////////////////////////////////////////////////////////////////

#define LEN_CALIB_STATE_LIST 3

typedef enum {
	CalibStateNull   = 0,
	CalibStateTraj_1 = 1,
	CalibStateTraj_2 = 2,
} calib_state_t;

// Calibration state strings (must match enumeration):
static char CALIB_STATE_STR[LEN_CALIB_STATE_LIST][LEN_STR_MAX] = {
	"CALIB STATE NULL",
	"CALIB STATE TRAJ 1",
	"CALIB STATE TRAJ 2"
};

///////////////////////////////////////////////////////////////////////////////
// Admittance control - parameters struct:
///////////////////////////////////////////////////////////////////////////////

typedef struct {
	float inertia_x;
	float inertia_y;
	float inertia_phi;
	float damping;
	float stiffness;
	float p_eq_x;
	float p_eq_y;
	float Fx_offset;
	float Fy_offset;
	float F_tang_magn;
} admitt_model_params_t;

admitt_model_params_t
set_admitt_model_params(	float inertia_x, float inertia_y,
							float damping, float stiffness,
							float p_eq_x, float p_eq_y);

#endif /* ALL_CODES_CONTROL_SETTINGS_LL2_ */


