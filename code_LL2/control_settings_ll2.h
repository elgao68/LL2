#ifndef CONTROL_SETTINGS_LL2_
#define CONTROL_SETTINGS_LL2_

///////////////////////////////////////////////////////////////////////////////
// General program settings (CRITICAL):
///////////////////////////////////////////////////////////////////////////////

// Motor torque activation:
#define MOTOR_TORQUE_ACTIVE_CALIB		1
#define MOTOR_TORQUE_ACTIVE_JOG			0
#define MOTOR_TORQUE_ACTIVE_EXERCISE	1

///////////////////////////////////////////////////////////////////////////////
// End-effector trajectory parameters - DEFAULT:
///////////////////////////////////////////////////////////////////////////////

#define CYCLE_PERIOD_DEF	3.0
#define EXP_BLEND_TIME		3.0 // 9.0
#define SEMIAXIS_X_DEF		0.15 // 0.12
#define SEMIAXIS_Y_DEF		0.075
#define ROT_ANGLE_DEF		0.0 // (-PI/6)
#define CYCLE_DIR_DEF		1
#define PHI_INIT_EXERC      (PI)

// Calibration parameters:
#define TEST_CALIB_RUN		0
#define V_CALIB				0.1
#define FRAC_RAMP_CALIB     0.1
#define DIST_CALIB_MAX      0.6

#define THR_DT_P_CONTACT    0.01
#define THR_VOLTAGE_CONTACT 0.8

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

#define IDX_SCALE_EXERCISE  0
#define IDX_SCALE_CALIB     1

// Scaling factors must be consistent with above indices:
static double SCALE_FB[]	  = {800.0,  80.0};
static double SCALE_FF[]	  = {  0.4,   0.0};
static double SCALE_GCOMP[]   = {  1.2,   0.0};
static double SCALE_INT_ERR[] = {  0.0,   0.0};

#define SCALE_F_END_MEAS  1.0

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
// Robot dimensions:
///////////////////////////////////////////////////////////////////////////////

// Workspace length:
#define D_WKSPC_LL2_X	0.46
#define D_WKSPC_LL2_Y	0.46

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
	"NoCtrl",
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
	NullTraj      = 0,
	EllipticTraj  = 1,
	LinearTraj    = 2,
	IsometricTraj = 3
} traj_type_t;

// Exercise mode strings (must match enumeration):
static char TRAJ_TYPE_STR[LEN_TRAJ_TYPE_LIST][LEN_STR_MAX] = {
	"NULL TRAJ",
	"ELLIPTIC TRAJ",
	"LINEAR TRAJ",
	"ISOMETRIC TRAJ"
};

///////////////////////////////////////////////////////////////////////////////
// Calibration states:
///////////////////////////////////////////////////////////////////////////////

#define LEN_CALIB_TRAJ_LIST 5

typedef enum {
	CalibTraj_Null				= 0,
	CalibTraj_1_Y_Travel		= 1,
	CalibTraj_2_X_Travel		= 2,
	CalibTraj_3_Travel_to_ORG	= 3,
	CalibTraj_4_Travel_to_P_Start_Exe = 4,
} calib_traj_t;

// Calibration state strings (must match enumeration):
static char CALIB_TRAJ_STR[LEN_CALIB_TRAJ_LIST][LEN_STR_MAX] = {
	"CALIB TRAJ NULL",
	"CALIB TRAJ 1 Y",
	"CALIB TRAJ 2 X",
	"CALIB TRAJ 3 TO ORG",
	"CALIB TRAJ 4 TO P START"
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

#endif /* ALL_CODES_CONTROL_SETTINGS_LL2_ */


