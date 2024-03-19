#ifndef ADMITT_CTRL_PARAMS
#define ADMITT_CTRL_PARAMS

#include "common.h"

typedef struct {
  float inertia_x;
  float inertia_y;
  float damping;
  float stiffness;
  float p_eq_x;
  float p_eq_y;
  float Fx_offset;
  float Fy_offset;
} admitt_model_params_t;

admitt_model_params_t
set_admitt_model_params(	float inertia_x, float inertia_y,
									float damping, float stiffness,
									float p_eq_x, float p_eq_y, float Fx_offset, float Fy_offset);

#endif
