/////////////////////////////////////////////////////////////////////////////
//
// motor_algo_ll2_help.c
//
// Created on: 2024.06.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <motor_algo_ll2.h>

///////////////////////////////////////////////////////////////////////////////
// Dynamic system parameters:
///////////////////////////////////////////////////////////////////////////////

admitt_model_params_t admitt_model_params_local;

///////////////////////////////////////////////////////////////////////////////
// Motor variables:
///////////////////////////////////////////////////////////////////////////////

static uint32_t ui32PkLMotorVoltDurationMs = 0; //for tracking the duration where the motor voltage remained at peak voltage
static uint32_t ui32PkRMotorVoltDurationMs = 0; //for tracking the duration where the motor voltage remained at peak voltage
static uint8_t  ui8MotorVoltSafetyState    = 0;

///////////////////////////////////////////////////////////////////////////////
// Control variables:
///////////////////////////////////////////////////////////////////////////////

static uint8_t ui8SafetyOff = 0; // 1 = safety off
static force_sensor_list_t force_sensor_calib_param;

///////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS, DYNAMIC SYSTEMS
///////////////////////////////////////////////////////////////////////////////

void
ode_admitt_model_nml(nml_mat* dt_z_nml, nml_mat* z_nml, ode_param_struct ode_params) {

    ///////////////////////////////////////////////////////////////////////////////
    // Declare static matrices:
    ///////////////////////////////////////////////////////////////////////////////

	static nml_mat* F_end_in_nml;

	static nml_mat* M_sys_q;
	static nml_mat* B_sys_q;
	static nml_mat* K_sys_q;

	static nml_mat* A_con;

	static nml_mat* Q_in_nml;
	static nml_mat* q_eq_nml;

	///////////////////////////////////////////////////////////////////////////////
    // Initialize matrices:
    ///////////////////////////////////////////////////////////////////////////////

	static int init_nml = 1;

	if (init_nml) {
		F_end_in_nml = nml_mat_new(  N_COORD_EXT, 1);

		M_sys_q  = nml_mat_new(N_COORD_EXT, N_COORD_EXT);
		B_sys_q  = nml_mat_new(N_COORD_EXT, N_COORD_EXT);
		K_sys_q  = nml_mat_new(N_COORD_EXT, N_COORD_EXT);

		// Constraint matrix:
		A_con    = nml_mat_new(N_CONSTR_TRAJ, N_COORD_EXT);

		Q_in_nml = nml_mat_new(  N_COORD_EXT, 1);
		q_eq_nml = nml_mat_new(  N_COORD_EXT, 1);

		init_nml = 0;
	}

	///////////////////////////////////////////////////////////////////////////////
	// Counters & constants:
	///////////////////////////////////////////////////////////////////////////////

	int c_i, r_i;
	static int step_i = 0;

	///////////////////////////////////////////////////////////////////////////////
	// System inputs:
	///////////////////////////////////////////////////////////////////////////////

	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++)
		F_end_in_nml->data[c_i][0] = ode_params.par_dbl[c_i];

	///////////////////////////////////////////////////////////////////////////////
	// Build system model matrices:
	///////////////////////////////////////////////////////////////////////////////

	double m_val[N_COORD_EXT];
	double b_val[N_COORD_EXT];
	double k_val[N_COORD_EXT];

	m_val[IDX_X  ] = (double) admitt_model_params_local.inertia_x;
	m_val[IDX_Y  ] = (double) admitt_model_params_local.inertia_y;
	m_val[IDX_PHI] = (double) admitt_model_params_local.inertia_phi;

	b_val[IDX_X  ] = (double) admitt_model_params_local.damping;
	b_val[IDX_Y  ] = (double) admitt_model_params_local.damping;
	b_val[IDX_PHI] = 0.0;

	k_val[IDX_X  ] = (double) admitt_model_params_local.stiffness;
	k_val[IDX_Y  ] = (double) admitt_model_params_local.stiffness;
	k_val[IDX_PHI] = 0.0;

	admitt_model_matrices_nml(M_sys_q, B_sys_q, K_sys_q, m_val, b_val, k_val, N_COORD_EXT);
	B_sys_q->data[IDX_PHI][IDX_PHI] = 1.0; // HACK: needed for stability

	///////////////////////////////////////////////////////////////////////////////
	// Get trajectory constraint matrix:
	///////////////////////////////////////////////////////////////////////////////

	nml_mat_cp_ref(A_con, ode_params.par_nml[IDX_PAR_NML_A_CON]);

	///////////////////////////////////////////////////////////////////////////////
	// Build additional matrices:
	///////////////////////////////////////////////////////////////////////////////

	double q_eq[N_COORD_EXT];

	q_eq[IDX_X  ] = (double) admitt_model_params_local.p_eq_x;
	q_eq[IDX_Y  ] = (double) admitt_model_params_local.p_eq_y;
	q_eq[IDX_PHI] = 0.0;

	for (c_i = IDX_X; c_i <= IDX_PHI; c_i++)
		q_eq_nml->data[c_i][0] = q_eq[c_i];

	///////////////////////////////////////////////////////////////////////////////
	// Obtain dynamic system output:
	///////////////////////////////////////////////////////////////////////////////

	if (ode_params.USE_ODE_CONSTR)
		dyn_sys_msd_nml_constr_lagr(
			dt_z_nml, Q_in_nml, z_nml, F_end_in_nml, M_sys_q, B_sys_q, K_sys_q, q_eq_nml, A_con);
	else
		dyn_sys_msd_nml_unc(
			dt_z_nml, Q_in_nml, z_nml, F_end_in_nml, M_sys_q, B_sys_q, K_sys_q, q_eq_nml);

	// ITM console output:
	#if USE_ITM_OUT_ADMITT_MODEL
		if (step_i == 0) {
			printf("\n");

			printf("M_sys_q = [ ...\n");
			for (r_i = 0; r_i < M_sys_q->num_rows; r_i++) {
				for (c_i = 0; c_i < M_sys_q->num_cols; c_i++) {
					printf("%f", M_sys_q->data[r_i][c_i]);
					if (c_i < (M_sys_q->num_cols - 1))
						printf(", ");
					else
						printf("\n");
				}
			}
			printf("];\n\n");

			printf("B_sys_q = [ ...\n");
			for (r_i = 0; r_i < B_sys_q->num_rows; r_i++) {
				for (c_i = 0; c_i < B_sys_q->num_cols; c_i++) {
					printf("%f", B_sys_q->data[r_i][c_i]);
					if (c_i < (B_sys_q->num_cols - 1))
						printf(", ");
					else
						printf("\n");
				}
			}
			printf("];\n\n");

			printf("K_sys_q = [ ...\n");
			for (r_i = 0; r_i < K_sys_q->num_rows; r_i++) {
				for (c_i = 0; c_i < K_sys_q->num_cols; c_i++) {
					printf("%f", K_sys_q->data[r_i][c_i]);
					if (c_i < (K_sys_q->num_cols - 1))
						printf(", ");
					else
						printf("\n");
				}
			}
			printf("];\n\n");

			printf("q_eq = [ ...\n");
			for (r_i = 0; r_i < q_eq_nml->num_rows; r_i++) {
				printf("%f", q_eq_nml->data[r_i][0]);
				if (r_i < (q_eq_nml->num_rows - 1))
					printf("; ");
				else
					printf("]\n");
			}
			printf("\n");
		}
		/*
		int DECIM_DISP_DT_Z = DECIM_DISP_GENERAL;

		if ((step_i % DECIM_DISP_DT_Z) == 0) {
			printf("____________________________\n");
			printf("ode_admitt_model_nml: [%d]\tF_end_in_nml = [%f\t%f]\t dt_z = [",
				step_i,
				F_end_in_nml->data[IDX_X][0], F_end_in_nml->data[IDX_Y][0]);

			for (c_i = 0; c_i < 2*N_COORD_EXT; c_i++) {
				printf("%f\t", dt_z_nml->data[c_i][0]);
			}
			printf("]\n\n");
		}
		*/
	#endif

	step_i++;
}

void
admitt_model_matrices_nml(nml_mat* M_sys, nml_mat* B_sys, nml_mat* K_sys, double m_val[], double b_val[], double k_val[], size_t N_q) {

	int r_i, c_i;
	for (r_i = 0; r_i < N_q; r_i++)
		for (c_i = 0; c_i < N_q; c_i++) {
			if (r_i == c_i) {
				M_sys->data[r_i][c_i] = m_val[c_i];
				B_sys->data[r_i][c_i] = b_val[c_i];
				K_sys->data[r_i][c_i] = k_val[c_i];
			}
			else {
				M_sys->data[r_i][c_i] = 0.0;
				B_sys->data[r_i][c_i] = 0.0;
				K_sys->data[r_i][c_i] = 0.0;
			}
		}
}

///////////////////////////////////////////////////////////////////////////////
// LOWER-LIMB UTILITY FUNCTIONS:
///////////////////////////////////////////////////////////////////////////////

uint8_t
set_LL_mech_readings(lowerlimb_mech_readings_t* mech, uint64_t current_upTime,
	int32_t L_count, int32_t R_count,
	uint32_t L_currsens, uint32_t R_currsens,
	uint32_t force_end_in_x, uint32_t force_end_in_y,
	uint8_t isCalibration) {

    float delta_time_s;
    uint8_t motor_result = 0;

	///////////////////////////////////////////////////////////////////////////////
    // check if need to init system
	///////////////////////////////////////////////////////////////////////////////

    if (mech->init == false) {
        ui32PkLMotorVoltDurationMs = 0;
        ui32PkRMotorVoltDurationMs = 0;
        ui8MotorVoltSafetyState    = 0;
        mech->init = true;
    }

    ///////////////////////////////////////////////////////////////////////////////
	// update prev units
	///////////////////////////////////////////////////////////////////////////////

    mech->prev_coord.x     = mech->coord.x;
    mech->prev_coord.y     = mech->coord.y;

    mech->prev_update_time = mech->update_time;
    mech->update_time      = current_upTime;

    mech->left.prev_angular_pos_rad  = mech->left.angular_pos_rad;
    mech->right.prev_angular_pos_rad = mech->right.angular_pos_rad;

    ///////////////////////////////////////////////////////////////////////////////
    // safety limiter for sampling time
    ///////////////////////////////////////////////////////////////////////////////

    // if (isCalibration == 0)
    //     safety_status = lowerlimb_motor_safety_limiter(INSUF_SAMPLING, 0);

    // if (safety_status == 1)
    //     motor_result == 0xF0;

    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // SENSOR READINGS
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////
    // update QEI counts
    ///////////////////////////////////////////////////////////////////////////////

    mech->left.qei_count  = L_count;
    mech->right.qei_count = R_count;

    ///////////////////////////////////////////////////////////////////////////////
    // compute angular displacement in rad - Motor Coordinates
    ///////////////////////////////////////////////////////////////////////////////

    mech-> left.angular_pos_rad = (float)mech-> left.qei_count * (float)2 * (float)M_PI / (float)MAX_QEI_CPR ;
    mech->right.angular_pos_rad = (float)mech->right.qei_count * (float)2 * (float)M_PI / (float)MAX_QEI_CPR;

    ///////////////////////////////////////////////////////////////////////////////
    // compute current coordinates in m / rad - End-effector Coordinates; Jacobian Matrix
    ///////////////////////////////////////////////////////////////////////////////

    mech->coord.x = (float)JACOBIAN * (  mech->left.angular_pos_rad - mech->right.angular_pos_rad);
    mech->coord.y = (float)JACOBIAN * (- mech->left.angular_pos_rad - mech->right.angular_pos_rad);

    ///////////////////////////////////////////////////////////////////////////////
    // compute angular velocity in rad/s - Angular Velocity at Motor
    ///////////////////////////////////////////////////////////////////////////////

    delta_time_s = (float)(mech->update_time - mech->prev_update_time) / (float)MSEC_PER_SEC; // in s

    if (delta_time_s == 0) {
    	mech-> left.angular_vel_rad = 0;
    	mech->right.angular_vel_rad = 0;
    }
    else {
    	mech-> left.angular_vel_rad = (mech-> left.angular_pos_rad - mech-> left.prev_angular_pos_rad) / delta_time_s;
    	mech->right.angular_vel_rad = (mech->right.angular_pos_rad - mech->right.prev_angular_pos_rad) / delta_time_s;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Compute velocity in m/s at end-effector:
    ///////////////////////////////////////////////////////////////////////////////

    mech->velocity.x = (float)JACOBIAN * (  mech->left.angular_vel_rad - mech->right.angular_vel_rad);
    mech->velocity.y = (float)JACOBIAN * (- mech->left.angular_vel_rad - mech->right.angular_vel_rad);

    ///////////////////////////////////////////////////////////////////////////////
    // Safety limiter check:
    ///////////////////////////////////////////////////////////////////////////////

    uint8_t safety_status = 0;

    if (isCalibration == 0)
    	safety_status = lowerlimb_motor_safety_limiter(RT_VEL_SAFETY, 0);

    if (safety_status == 1)
        motor_result = 0xF1;

    ///////////////////////////////////////////////////////////////////////////////
    //  Compute current sensor voltage:
    ///////////////////////////////////////////////////////////////////////////////

    if (L_currsens >= 0 && L_currsens <= 4095){
    	mech->left.currsens_volt = L_currsens * 3.3 / 4095;
    	mech->left.currsens_amps = (mech->left.currsens_volt * MD_IN_CURRVOLT_GRADIENT) +  MD_IN_VOLT_OFFSET;
    }
    else {
    	mech->left.currsens_volt = -1;
    	mech->left.currsens_amps = -1;
    }

    if (R_currsens >= 0 && R_currsens <= 4095){
    	mech->right.currsens_volt = R_currsens * 3.3 / 4095;
    	mech->right.currsens_amps = (mech->right.currsens_volt * MD_IN_CURRVOLT_GRADIENT) +  MD_IN_VOLT_OFFSET;
    }
    else {
    	mech->right.currsens_volt = -1;
    	mech->right.currsens_amps = -1;
    }

    ///////////////////////////////////////////////////////////////////////////////
    //  Compute end-effector's exerted force:
    ///////////////////////////////////////////////////////////////////////////////

    mech->Xforce_volt = force_end_in_x * 3.3f / 4095.0f;
    mech->Yforce_volt = force_end_in_y * 3.3f / 4095.0f;

    mech->Xforce_volt_corrected = mech->Xforce_volt + force_sensor_calib_param.X_axis;
    mech->Yforce_volt_corrected = mech->Yforce_volt + force_sensor_calib_param.Y_axis;

    mech->Xforce = mech->Xforce_volt_corrected * X_FORCE_VOLT_GRADIENT + X_FORCE_VOLT_OFFSET;
    mech->Yforce = mech->Yforce_volt_corrected * Y_FORCE_VOLT_GRADIENT + Y_FORCE_VOLT_OFFSET;

	///////////////////////////////////////////////////////////////////////////////
	// Update motor alert:
	///////////////////////////////////////////////////////////////////////////////

	uint8_t motor_alert;

	if (motor_result == 0xF0)
		motor_alert = 1;
	else if (motor_result == 0xF1)
		motor_alert = 2;
	else
		motor_alert = 0;

	return motor_alert;
}

void
set_LL_motor_settings(lowerlimb_motors_settings_t* motors, float force_end[]) {

	///////////////////////////////////////////////////////////////////////////////
	// Adding offset forces
	///////////////////////////////////////////////////////////////////////////////

	motors->force_end[IDX_X] = force_end[IDX_X];
	motors->force_end[IDX_Y] = force_end[IDX_Y];

	///////////////////////////////////////////////////////////////////////////////
	// Compute torque in N-m
	///////////////////////////////////////////////////////////////////////////////

	motors->left.torque  = JACOBIAN * ( motors->force_end[IDX_X] - motors->force_end[IDX_Y]);
	motors->right.torque = JACOBIAN * (-motors->force_end[IDX_X] - motors->force_end[IDX_Y]);

	///////////////////////////////////////////////////////////////////////////////
	// Compute current in A
	///////////////////////////////////////////////////////////////////////////////

	motors->left.current  = motors->left.torque  / (float) MD_T_CONST;
	motors->right.current = motors->right.torque / (float) MD_T_CONST;

	///////////////////////////////////////////////////////////////////////////////
	// Calculate Commanded Motor Driver Voltage
	///////////////////////////////////////////////////////////////////////////////

	motors->left.volt  = (MD_OUT_CURRVOLT_GRADIENT * fabs(motors->left.current) ) + MD_OUT_VOLT_OFFSET;
	motors->right.volt = (MD_OUT_CURRVOLT_GRADIENT * fabs(motors->right.current)) + MD_OUT_VOLT_OFFSET;

	///////////////////////////////////////////////////////////////////////////////
	// Check Commanded Motor Driver Voltage and convert to DAC value 0 to 4095
	///////////////////////////////////////////////////////////////////////////////

	if (motors->left.volt > MD_MAX_OUT_VOLT)
		motors->left.volt = MD_MAX_OUT_VOLT;
	else if (motors->left.volt < MD_MIN_OUT_VOLT)
		motors->left.volt = MD_MIN_OUT_VOLT;

	if (motors->right.volt > MD_MAX_OUT_VOLT)
		motors->right.volt = MD_MAX_OUT_VOLT;
	else if (motors->right.volt < MD_MIN_OUT_VOLT)
		motors->right.volt = MD_MIN_OUT_VOLT;

	motors->left.dac_in  = (uint32_t) roundf(motors->left.volt  / 3.3 * 4095.0);
	motors->right.dac_in = (uint32_t) roundf(motors->right.volt / 3.3 * 4095.0);

	// Check directions:
	if (motors->left.torque > 0) {
		motors->left.motor_direction = false;
		motors->left.en_motor_driver = true;
	}
	else if (motors->left.torque == 0) //free wheeling
		motors->right.en_motor_driver = false;
	else {
		motors->left.motor_direction = true;
		motors->left.en_motor_driver = true;
	}

	if (motors->right.torque > 0) {
		motors->right.motor_direction = false;
		motors->right.en_motor_driver = true;
	}
	else if (motors->right.torque == 0) // free wheeling
		motors->right.en_motor_driver = false;
	else {
		motors->right.motor_direction = true;
		motors->right.en_motor_driver = true;
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS, LOW LEVEL
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Safety functions:
///////////////////////////////////////////////////////////////////////////////

// @param[in]: en: 1 = disable safety, 0 = enable safety
void
set_safetyOff(uint8_t en) {
    //limit
    en = en > 1? 1 : en;
    ui8SafetyOff = en;
}

uint8_t
get_safetyOff(void) {
    return ui8SafetyOff;
}

///////////////////////////////////////////////////////////////////////////////
// Sensor readings:
///////////////////////////////////////////////////////////////////////////////

void
clear_lowerlimb_mech_readings(lowerlimb_mech_readings_t* LL_mech_readings)
{
    memset(LL_mech_readings, 0, sizeof(lowerlimb_mech_readings_t));
}

///////////////////////////////////////////////////////////////////////////////
// Motor settings:
///////////////////////////////////////////////////////////////////////////////

void
clear_lowerlimb_motors_settings(lowerlimb_motors_settings_t* LL_motors_settings)
{
    memset(LL_motors_settings, 0, sizeof(lowerlimb_motors_settings_t));
}

uint32_t
get_motor_L_dac(lowerlimb_motors_settings_t* LL_motors_settings)
{
	return LL_motors_settings->left.dac_in;
}

uint32_t
get_motor_R_dac(lowerlimb_motors_settings_t* LL_motors_settings)
{
	return LL_motors_settings->right.dac_in;
}

bool
get_motor_L_direction(lowerlimb_motors_settings_t* LL_motors_settings)
{
	return LL_motors_settings->left.motor_direction;
}

bool
get_motor_R_direction(lowerlimb_motors_settings_t* LL_motors_settings)
{
	return LL_motors_settings->right.motor_direction;
}

bool
get_motor_L_enable(lowerlimb_motors_settings_t* LL_motors_settings)
{
	return LL_motors_settings->left.en_motor_driver;
}

bool
get_motor_R_enable(lowerlimb_motors_settings_t* LL_motors_settings)
{
	return LL_motors_settings->right.en_motor_driver;
}

///////////////////////////////////////////////////////////////////////////////
// Exercise functions:
///////////////////////////////////////////////////////////////////////////////

void
init_motor_algo(lowerlimb_mech_readings_t* LL_mech_readings, lowerlimb_motors_settings_t* LL_motors_settings)
{
    clear_lowerlimb_motors_settings(LL_motors_settings); // CRITICAL: if unchecked it could cause encoders to reset unexpectedly
    clear_lowerlimb_mech_readings(LL_mech_readings);
    // clear_transition_mode_params();
    
    ui8SafetyOff = 0;
}

uint8_t
lowerlimb_motor_safety_limiter(uint8_t type, uint8_t targ_index)
{
#if (lowerlimb_SAFETY)
    uint8_t status = 0;
    uint32_t deltaSampleMs = 1; //default sample time
    if(LL_mech_readings.prev_update_time != 0)
        deltaSampleMs = (uint32_t)(LL_mech_readings.update_time - LL_mech_readings.prev_update_time);

    //safety has been disabled
    if(ui8SafetyOff == 1)
        return 0xff;

    switch(type)
    {
    case INSUF_SAMPLING:
        if(deltaSampleMs > lowerlimb_SAMPLING_MAX)
        {
            status = 1;
#if (DEBUG_PRINT)
            lowerlimb_printf("Safety limiter: Insufficient sampling. Delta ms = %d\r\n", deltaSampleMs);
#endif
        }
        break;
    case RT_VEL_SAFETY:

//        if (LL_mech_readings.sqrt_velocity >= (float)lowerlimb_RT_MAX_VEL)
//        {
//            status = 1;
//
//            //sending warning?
//#if (DEBUG_PRINT)
//            lowerlimb_printf("Safety limiter: RT velocity safety.\r\n");
//#endif
//        }

        break;

    default:
        status = 2; //invalid safety type
        break;
    }

    return status; //return status
#else
    return 0xff;
#endif
}

///////////////////////////////////////////////////////////////////////////////
// Offset forces:
///////////////////////////////////////////////////////////////////////////////

void set_force_sensor_zero_offset (float X_axis, float Y_axis) {
  force_sensor_calib_param.X_axis = 1.8f - X_axis;
  force_sensor_calib_param.Y_axis = 1.8f - Y_axis;
}

