#ifndef COMMON_H_
#define COMMON_H_

typedef struct{
  float x;
  float y;
} cartesian_coord;

typedef enum {
  ImpedanceCtrl = 1,
  PassiveTrajectoryCtrl = 2,
  AdmittanceCtrl = 3,
  ActiveTrajectoryCtrl = 4,
} exercise_mode_t;

#endif /* ALL_CODES_COMMON_H_ */
