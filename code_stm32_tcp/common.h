#ifndef COMMON_H_
#define COMMON_H_

typedef struct{
  float x;
  float y;
} cartesian_coord;

#define LEN_STR_MAX   35
#define LEN_EXERC_MODE_LIST 5

typedef enum {
	ImpedanceCtrl         = 1,
	PassiveTrajectoryCtrl = 2,
	AdmittanceCtrl        = 3,
	ActiveTrajectoryCtrl  = 4,
} exercise_mode_t;

// Exercise mode strings (must match exercise modes enumeration):
static char EXERC_MODE_STR[LEN_EXERC_MODE_LIST][LEN_STR_MAX] = {
	"",
	"ImpedanceCtrl",
	"PassiveTrajectoryCtrl",
	"AdmittanceCtrl",
	"ActiveTrajectoryCtrl"
};

#endif /* ALL_CODES_COMMON_H_ */
