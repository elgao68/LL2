/////////////////////////////////////////////////////////////////////////////
//
// sensors_ll2.c
//
// Created on: 2024.07.08
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <sensors_ll2.h>

void
force_sensors_zero_calibrate(ADC_HandleTypeDef* hadc3) {
	uint32_t force_end_in_x = 0;
	uint32_t force_end_in_y = 0;
	uint32_t dum_force_end_in_x    = 0;
	uint32_t dum_force_end_in_y    = 0;
	float force_end_in_x_f  = 0;
	float force_end_in_y_f  = 0;

	const uint8_t N_SENSOR_READS = 50;

	// Zero-calibrate force sensors:
	for (int i = 1; i <= N_SENSOR_READS; i++) {
		force_sensors_read(hadc3, &force_end_in_x, &force_end_in_y,
				&dum_force_end_in_x, &dum_force_end_in_y);

		force_end_in_x_f += (float) force_end_in_x * 3.3f / 4095.0f; // TODO: what do the magic numbers mean?
		force_end_in_y_f += (float) force_end_in_y * 3.3f / 4095.0f;
	}

	force_end_in_x_f = force_end_in_x_f / 50.0f; // TODO: what do the magic numbers mean?
	force_end_in_y_f = force_end_in_y_f / 50.0f;

	set_force_sensor_zero_offset(force_end_in_x_f, force_end_in_y_f);
}
