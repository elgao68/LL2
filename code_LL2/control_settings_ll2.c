#include <control_settings_ll2.h>


admitt_model_params_t
set_admitt_model_params(	float inertia_x, float inertia_y,
									float damping, float stiffness,
									float p_eq_x, float p_eq_y, float Fx_offset, float Fy_offset) {

	admitt_model_params_t admitt_model_params;

	admitt_model_params.inertia_x = inertia_x;
	admitt_model_params.inertia_y = inertia_y;
	admitt_model_params.damping   = damping;
	admitt_model_params.stiffness = stiffness;
	admitt_model_params.p_eq_x    = p_eq_x;
	admitt_model_params.p_eq_y    = p_eq_y;
	admitt_model_params.Fx_offset = Fx_offset;
	admitt_model_params.Fy_offset = Fy_offset;

	return admitt_model_params;
}

