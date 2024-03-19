/////////////////////////////////////////////////////////////////////////////
//
// main.c
//
// Created on: 2024.02.10
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#include <_std_c.h>
#include <admitt_model_params.h>
#include <lowerlimb_tcp_app.h>
#include <motor_algo_ll2.h>
#include <nml.h>
#include <nml_util.h>
#include <traj_ctrl_params_nml.h>

#include <string.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "timer.h"
#include "uart_driver.h"
#include "qei_motor_drivers.h"
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "w5500_app.h"
#include "peripheral.h"
#include "_params_simulation.h"
// #include "lowerlimb_calib_protocols.h"

////////////////////////////////////////////////////////////////////////////////
// MEMORY HANDLING - CRITICAL - GAO
////////////////////////////////////////////////////////////////////////////////

#define HEAP_SIZE_MIN	3840
#define KBYTE           1024

////////////////////////////////////////////////////////////////////////////////
// TEST SCRIPTS - DECLARATIONS - GAO
////////////////////////////////////////////////////////////////////////////////

void test_real_time();
void test_simulation();
void test_scratch();

////////////////////////////////////////////////////////////////////////////////
// Private variables:
////////////////////////////////////////////////////////////////////////////////

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DAC_HandleTypeDef hdac;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
UART_HandleTypeDef huart3;

////////////////////////////////////////////////////////////////////////////////
// USER CODE BEGIN PV
////////////////////////////////////////////////////////////////////////////////

// TCP ethernet related
//  Demo Firmware Version
#define VER_H		0x00
#define VER_L		0x00
#define VER_P		0x00
#define INTERVAL_5MS		5
uint64_t ethernet_test_nextTime = 0;

static uint64_t algo_nextTime = 0;
static uint64_t brakes_nextTime = 0;
static lowerlimb_sys_info_t LL_sys_info;
static uint64_t expire_nextTime = 0;
static uint8_t prev_fifo_size = 0;

// ADC
static uint32_t dum_force_end_in_x = 0;
static uint32_t dum_force_end_in_y = 0;
static uint32_t current_sensor_L = 0;
static uint32_t current_sensor_R = 0;

// Motor driver feedback related:
// static uint8_t tcpTxData[292];
// static uint64_t R_brakes_powersavetimer = 0;
// static uint64_t P_brakes_powersavetimer = 0;

////////////////////////////////////////////////////////////////////////////////
// Private function prototypes:
////////////////////////////////////////////////////////////////////////////////

void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_GPIO_Init(void);

void _Error_Handler(char *file, int line);

////////////////////////////////////////////////////////////////////////////
// CONTROL / SIMULATION SETTINGS:
/////////////////////////////////////////////////////////////////////////////

#define _TEST_DEBUG_         0

#define _TEST_REAL_TIME      1
#define _TEST_SIMULATION	 2
#define _TEST_SCRATCH        3

#define TEST_OPTION			 _TEST_REAL_TIME

// Dynamic system mode: unconstrained / constrained:
#define USE_ADMITT_MODEL_CONSTR		0
#define OVERRIDE_DYN_PARAMS			1

#define DT_DISP_MSEC_GUI_PARAMS	2000
#define DT_DISP_MSEC_REALTIME	1000

#define USE_ITM_OUT_GUI_PARAMS	0
#define USE_ITM_OUT_REAL_TIME	0
#define USE_ITM_OUT_SIM			0

#define DT_EXPIRE_MSEC		 	1000

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS - DECLARATIONS - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int _write(int32_t file, uint8_t *ptr, int32_t len);
// int __io_putchar(int ch);

void cycle_haptic_buttons();
void LED_sys_state_off();
void set_brakes_timed(uint64_t uptime, uint64_t* brakes_next_time);
uint8_t set_LL_exercise_feedback_help(lowerlimb_mech_readings_t* mech_readings, lowerlimb_motors_settings_t* motor_settings);

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// MAIN FUNCTION - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int
main(void) {

	/////////////////////////////////////////////////////////////////////////////////////
	// 'Defragment' the heap - TODO: remove at a later date
	/////////////////////////////////////////////////////////////////////////////////////

	// uint8_t heap = (uint8_t*)malloc(HEAP_SIZE_MIN*KBYTE*sizeof(uint8_t));
	// free(heap);

	/////////////////////////////////////////////////////////////////////////////////////
	// Reset of all peripherals, Initializes the Flash interface and the Sys tick.
	/////////////////////////////////////////////////////////////////////////////////////

	HAL_Init();

	/////////////////////////////////////////////////////////////////////////////////////
	// Configure the system clock
	/////////////////////////////////////////////////////////////////////////////////////

	SystemClock_Config();

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize all configured peripherals
	/////////////////////////////////////////////////////////////////////////////////////

	MX_GPIO_Init();
	MX_TIM9_Init();
	MX_USART3_UART_Init();
	MX_TIM4_Init();
	MX_SPI3_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_DAC_Init();

	/////////////////////////////////////////////////////////////////////////////////////
	// Check if you're only running "scratch" code:
	/////////////////////////////////////////////////////////////////////////////////////

#if TEST_OPTION == _TEST_SCRATCH
	test_scratch();
	return 0;
#endif

	/////////////////////////////////////////////////////////////////////////////////////
	//start UART sys
	/////////////////////////////////////////////////////////////////////////////////////

	uint8_t startup_status = 0;

	startup_status = uart_sys_init();
	// uart_printf("System starting up!\r\n");

	/////////////////////////////////////////////////////////////////////////////////////
	//start 1ms timer
	/////////////////////////////////////////////////////////////////////////////////////

	startup_status = startBaseTimer();
	/*
	if (startup_status) {
		uart_printf("Base Timer init err!\r\n");
	}
	*/

	/////////////////////////////////////////////////////////////////////////////////////
	//start motor driver
	/////////////////////////////////////////////////////////////////////////////////////

	startup_status = motor_qei_sys_start();

	/*
	if (startup_status) {
		uart_printf("Motor PWM and QEI init err!\r\n");
	}
	uart_printf("System startup success!\r\n");
	*/

	/////////////////////////////////////////////////////////////////////////////////////
	//disable motor
	/////////////////////////////////////////////////////////////////////////////////////

	motor_R_move(0, false, false);
	motor_L_move(0, false, false);

	/////////////////////////////////////////////////////////////////////////////////////
	// Set up Ethernet:
	/////////////////////////////////////////////////////////////////////////////////////

	Ethernet_Reset(true);
	HAL_Delay(100);
	Ethernet_Reset(false);
	HAL_Delay(1000);
	set_ethernet_w5500_mac(0x00, 0x0a, 0xdc, 0xab, 0xcd, 0xef);
	ethernet_w5500_sys_init();

#if _TEST_DEBUG_
	uint8_t tmpstr[6] = { 0, };

	ctlwizchip(CW_GET_ID, (void*) tmpstr);

	u@rt_printf("\r\n=======================================\r\n");
	u@rt_printf(" WIZnet %s Test code v%d.%.2d\r\n", tmpstr, VER_H, VER_L);
	u@rt_printf("=======================================\r\n");
	u@rt_printf(">> TCP client + Motor drivers test\r\n");
	u@rt_printf("=======================================\r\n");

	Display_Net_Conf(); //  Print out the network information to serial terminal
#endif

	/////////////////////////////////////////////////////////////////////////////////////
	// TCP APP
	/////////////////////////////////////////////////////////////////////////////////////

	// lowerlimb_tcp_init_app_state(0, VER_H, VER_L, VER_P, &LL_motors_settings); // TODO: remove at a later date

	/////////////////////////////////////////////////////////////////////////////////////
	// test MSB (REMOVED)
	/////////////////////////////////////////////////////////////////////////////////////

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	/////////////////////////////////////////////////////////////////////////////////////
	// Cycle LED
	/////////////////////////////////////////////////////////////////////////////////////

	Cycle_LED_Init();
	LED_sys_state_off();

	/////////////////////////////////////////////////////////////////////////////////////
	// Launch test script:
	/////////////////////////////////////////////////////////////////////////////////////

#if TEST_OPTION == _TEST_REAL_TIME
	test_real_time();
#elif TEST_OPTION == _TEST_SIMULATION
	test_simulation();
#endif

	return 0;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// TEST SCRIPT - REAL-TIME- GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_real_time() {

	/////////////////////////////////////////////////////////////////////////////////////
	// MOTOR STATE VARS - GAO:
	/////////////////////////////////////////////////////////////////////////////////////

	const int MOTOR_TORQUE_ON = 0;

	uint8_t motor_result      = 0;
	uint8_t motor_alert       = 0;
	uint8_t prevConnected     = 0;

	/////////////////////////////////////////////////////////////////////////////////////
	// FIRMWARE/CONTROL PARAMETERS - GAO:
	/////////////////////////////////////////////////////////////////////////////////////

	uint64_t up_time;
	uint64_t up_time_end;

	lowerlimb_mech_readings_t   LL_mech_readings;
	lowerlimb_motors_settings_t LL_motors_settings;
	traj_ctrl_params_t          traj_ctrl_params;
	admitt_model_params_t       admitt_model_params;

	const int IS_CALIBRATION = 1; // what did this flag do in update_motor_algo() (now set_LL_mech_readings())?

	/////////////////////////////////////////////////////////////////////////////////////
	// Sensor variables:
	/////////////////////////////////////////////////////////////////////////////////////

	// Force sensor readings, raw:
	uint32_t force_end_in_x_sensor = 0;
	uint32_t force_end_in_y_sensor = 0;

	// End-effector force measurements:
	double F_end_m[N_COORD_2D];

	/////////////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - REFERENCE:
	/////////////////////////////////////////////////////////////////////////////////////

    // Integrator time step:
	double dt_k = (double)DT_STEP_MSEC/MSEC_PER_SEC; // integrator step (initial)

	// Reference position and velocity:
	double    p_ref[N_COORD_2D] = {0.0, 0.0};
	double dt_p_ref[N_COORD_2D] = {0.0, 0.0};

	// Cycle phase and instantaneous frequency:
	double    phi_ref = 0.0;
	double dt_phi_ref = 0.0;

	// Trajectory path tangent vector:
	double u_t_ref[N_COORD_2D] = {0.0, 0.0};

	/////////////////////////////////////////////////////////////////////////////////////
	// Kinematics variables - MEASURED:
	/////////////////////////////////////////////////////////////////////////////////////

	// Measured position and velocity:
	double    p_m[N_COORD_2D];
	double dt_p_m[N_COORD_2D];

	/////////////////////////////////////////////////////////////////////////////////////
	// Counters:
	/////////////////////////////////////////////////////////////////////////////////////

	int step_rt = 0;

	/////////////////////////////////////////////////////////////////////////////////////
	// Force command:
	/////////////////////////////////////////////////////////////////////////////////////

	float force_end_cmd[N_COORD_2D] = {0.0, 0.0};

	/////////////////////////////////////////////////////////////////////////////////////
	// TCP app:
	/////////////////////////////////////////////////////////////////////////////////////

	lowerlimb_tcp_init_app_state(0, VER_H, VER_L, VER_P, &LL_motors_settings);

	/////////////////////////////////////////////////////////////////////////////////////
	// Initialize motor algorithm:
	/////////////////////////////////////////////////////////////////////////////////////

	init_motor_algo(&LL_mech_readings, &LL_motors_settings);

	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////
	// USER CODE BEGIN WHILE - GAO
	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////

	while (1) {

		/////////////////////////////////////////////////////////////////////////////////////
		// uart rx state check
		/////////////////////////////////////////////////////////////////////////////////////

		uart_rx_data_state();

		/////////////////////////////////////////////////////////////////////////////////////
		// ethernet check
		/////////////////////////////////////////////////////////////////////////////////////

		ethernet_w5500_state();

		////////////////////////////////////////////////////////////////////////////////////////
		// GET TCP/IP APP STATE - GAO
		////////////////////////////////////////////////////////////////////////////////////////

		LL_sys_info = lowerlimb_tcp_app_state(Read_Haptic_Button(), motor_alert,
				&traj_ctrl_params, &admitt_model_params, &LL_motors_settings);

		/////////////////////////////////////////////////////////////////////////////////////
		// Clear motor_alert after sending into tcp app state:
		/////////////////////////////////////////////////////////////////////////////////////

		motor_alert = 0;

		////////////////////////////////////////////////////////////////////////////////////////
		// Override trajectory and control parameters - TODO: remove at a later date
		////////////////////////////////////////////////////////////////////////////////////////

#if OVERRIDE_DYN_PARAMS
		/*
		traj_ctrl_params.cycle_period = 3.0;
		traj_ctrl_params.exp_blend_time = 3.0;
		traj_ctrl_params.semiaxis_x = 0.15;
		traj_ctrl_params.semiaxis_y = 0.08;
		traj_ctrl_params.rot_angle = 0;
		// traj_ctrl_params.cycle_dir = 1;
		*/

		admitt_model_params.inertia_x = 10;
		admitt_model_params.inertia_y = 10;
		admitt_model_params.damping   = 20;
		admitt_model_params.stiffness = 100;
		admitt_model_params.p_eq_x = 0;
		admitt_model_params.p_eq_y = 0;
		admitt_model_params.Fx_offset = 0;
		admitt_model_params.Fy_offset = 0;
#endif

#if USE_ITM_OUT_GUI_PARAMS
		if (step_rt > 0 && step_rt % (DT_DISP_MSEC_GUI_PARAMS/(int)(1000*dt_k)) == 0) {
			printf("___________________________________\n");
			printf("step_rt   = [%d]\n", step_rt);
			printf("\n");
			printf("cycle_period   = [%f]\n", traj_ctrl_params.cycle_period);
			printf("exp_blend_time = [%f]\n", traj_ctrl_params.exp_blend_time );
			printf("semiaxis_x     = [%f]\n", traj_ctrl_params.semiaxis_x);
			printf("semiaxis_y     = [%f]\n", traj_ctrl_params.semiaxis_y);
			printf("rot_angle      = [%f]\n", traj_ctrl_params.rot_angle);
			// printf("cycle_dir    = [%d]\n", traj_ctrl_params.cycle_dir);

			printf("\n");
			printf("inertia_x = [%f]\n", admitt_model_params.inertia_x );
			printf("inertia_y = [%f]\n", admitt_model_params.inertia_y);
			printf("damping   = [%f]\n", admitt_model_params.damping);
			printf("stiffness = [%f]\n", admitt_model_params.stiffness);
			printf("p_eq_x    = [%f]\n", admitt_model_params.p_eq_x );
			printf("p_eq_y    = [%f]\n", admitt_model_params.p_eq_y);
			printf("Fx_offset = [%f]\n", admitt_model_params.Fx_offset);
			printf("Fy_offset = [%f]\n", admitt_model_params.Fy_offset);
			printf("\n");

			fflush(stdout);
		}
#endif

		/////////////////////////////////////////////////////////////////////////////////////
		// Execute real-time step:
		/////////////////////////////////////////////////////////////////////////////////////

		up_time = getUpTime();

		if (up_time >= algo_nextTime) {

			algo_nextTime = up_time + DT_STEP_MSEC;

			/////////////////////////////////////////////////////////////////////////////////////
			// if system is on or off?
			/////////////////////////////////////////////////////////////////////////////////////

			if (LL_sys_info.system_state == ON) {

				/////////////////////////////////////////////////////////////////////////////////////
				// Check emergency signal GPIOG GPIO_PIN_14
				/////////////////////////////////////////////////////////////////////////////////////

				cycle_haptic_buttons();
				set_brakes_timed(up_time, &brakes_nextTime);

				/////////////////////////////////////////////////////////////////////////////////////
				// update safety
				/////////////////////////////////////////////////////////////////////////////////////

				set_safetyOff(LL_sys_info.safetyOFF);

				/////////////////////////////////////////////////////////////////////////////////////
				/////////////////////////////////////////////////////////////////////////////////////
				// SWITCH ACTIVITY STATE - GAO
				/////////////////////////////////////////////////////////////////////////////////////
				/////////////////////////////////////////////////////////////////////////////////////

				switch (LL_sys_info.activity_state) {

					/////////////////////////////////////////////////////////////////////////////////////
					// IDLE state:
					/////////////////////////////////////////////////////////////////////////////////////

					case IDLE:
						// clear motor algo readings and settings
						clear_lowerlimb_mech_readings(&LL_mech_readings);
						clear_lowerlimb_motors_settings(&LL_motors_settings);
						clear_ctrl_params();

						// disable motor
						motor_R_move(0, false, false);
						motor_L_move(0, false, false);

						// restart
						init_motor_algo(&LL_mech_readings, &LL_motors_settings);

						// Reset calibration state:
						// reset_calibration_state();

						break;

					/////////////////////////////////////////////////////////////////////////////////////
					// CALIBRATION state:
					/////////////////////////////////////////////////////////////////////////////////////

					case CALIB: // To-do (REMOVED)
						break;

					/////////////////////////////////////////////////////////////////////////////////////
					// EXERCISE state:
					/////////////////////////////////////////////////////////////////////////////////////

					case EXERCISE:
							if (LL_sys_info.exercise_state == RUNNING) {

								/////////////////////////////////////////////////////////////////////////////////////
								// Retrieve force sensor readings:
								/////////////////////////////////////////////////////////////////////////////////////

								force_sensors_read(&hadc3, &force_end_in_x_sensor, &force_end_in_y_sensor, &dum_force_end_in_x, &dum_force_end_in_y);
								current_sensors_read(&hadc1, &current_sensor_L, &current_sensor_R);

								/////////////////////////////////////////////////////////////////////////////////////
							    // Get lower-limb robot sensor readings:
								/////////////////////////////////////////////////////////////////////////////////////

								motor_alert = set_LL_mech_readings(&LL_mech_readings, up_time,
										qei_count_L_read(), qei_count_R_read(),
										current_sensor_L, current_sensor_R,
										force_end_in_x_sensor, force_end_in_y_sensor,
										IS_CALIBRATION);

								/////////////////////////////////////////////////////////////////////////////////////
								// Extract measured end-effector forces:
								/////////////////////////////////////////////////////////////////////////////////////

								F_end_m[IDX_X] = (double)LL_mech_readings.Xforce;
								F_end_m[IDX_Y] = (double)LL_mech_readings.Yforce;

								/////////////////////////////////////////////////////////////////////////////////////
								// Extract measured position and velocity from sensor readings:
								/////////////////////////////////////////////////////////////////////////////////////

								p_m[IDX_X]     = (double)LL_mech_readings.coord.x;
								p_m[IDX_Y]     = (double)LL_mech_readings.coord.y;

								dt_p_m[IDX_X]  = (double)LL_mech_readings.velocity.x;
								dt_p_m[IDX_Y]  = (double)LL_mech_readings.velocity.y;

								/////////////////////////////////////////////////////////////////////////////////////
								// Motor algorithm computation:
								/////////////////////////////////////////////////////////////////////////////////////

								traj_reference_step_active(p_ref, dt_p_ref, &phi_ref, &dt_phi_ref, u_t_ref, dt_k,
										F_end_m,
										traj_ctrl_params, admitt_model_params, USE_ADMITT_MODEL_CONSTR);

								/////////////////////////////////////////////////////////////////////////////////////
								// UPDATE MOTOR SETTINGS - GAO:
								/////////////////////////////////////////////////////////////////////////////////////

								// Clear motors settings:
							    clear_lowerlimb_motors_settings(&LL_motors_settings);

								set_LL_motor_settings(&LL_motors_settings, force_end_cmd);

								/////////////////////////////////////////////////////////////////////////////////////
								// Send motor commands:
								/////////////////////////////////////////////////////////////////////////////////////

								if (MOTOR_TORQUE_ON) {
									// Check if need to end exercise:
									if ((motor_alert == 1) || (motor_alert == 2)) {
										stop_exercise(&LL_motors_settings);

										// Disable motors:
										motor_R_move(0, false, false);
										motor_L_move(0, false, false);
									}
									else {
										motor_R_move(LL_motors_settings.right.dac_in, LL_motors_settings.right.motor_direction,
											LL_motors_settings.right.en_motor_driver);
										motor_L_move(LL_motors_settings.left.dac_in, LL_motors_settings.left.motor_direction,
											LL_motors_settings.left.en_motor_driver);
									}
								}

								/////////////////////////////////////////////////////////////////////////////////////
								// Distal Force Sensor - Change only when updating TCP Protocol
								// Input Brakes info from TCP System Info
								/////////////////////////////////////////////////////////////////////////////////////

								set_LL_exercise_feedback_help(&LL_mech_readings, &LL_motors_settings);

								up_time_end = getUpTime();

								// ITM console output:
#if USE_ITM_OUT_REAL_TIME
								if ((up_time_end - up_time) > DT_STEP_MSEC)
									printf("%d\t%f\t(%d)\n",
										step_rt,
										dt_k*step_rt,
										(int)(up_time_end - up_time));

								else if (step_rt % (DT_DISP_MSEC_REALTIME/(int)(1000*dt_k)) == 0)
									printf("%d\t%f\t(%d)\t%f\t%f\t%f\t%f\t%f\t%f\n",
										step_rt,
										dt_k*step_rt,
										(int)(up_time_end - up_time),
										phi_ref,
										dt_phi_ref,
										p_ref[IDX_X],
										p_ref[IDX_Y],
										dt_p_ref[IDX_X],
										dt_p_ref[IDX_Y]);
#endif

								step_rt++;
							}

							// Case when LL_sys_info.exercise_state != RUNNING:
							else {
								// reset
								clear_lowerlimb_mech_readings(&LL_mech_readings);
								clear_lowerlimb_motors_settings(&LL_motors_settings);
								clear_ctrl_params();

								// Set the motors to 0 and disable the motor driver
								motor_R_move(0, false, false);
								motor_L_move(0, false, false);

							} // end if (LL_sys_info.exercise_state == RUNNING)
					} // switch (LL_sys_info.activity_state)
			} // if (LL_sys_info.system_state == ON)
			else {
				LED_sys_state_off();
			}

			/////////////////////////////////////////////////////////////////////////////////////
			// indicate system was TCP connected
			/////////////////////////////////////////////////////////////////////////////////////

			prevConnected = 1;

			/////////////////////////////////////////////////////////////////////////////////////
			// Check if need to dump UART FIFO
			/////////////////////////////////////////////////////////////////////////////////////

			if (prev_fifo_size != rx_fifo_size()) {
				prev_fifo_size = rx_fifo_size();
				expire_nextTime = up_time + DT_EXPIRE_MSEC;
			}

			if ((up_time >= expire_nextTime) && (rx_fifo_size() > 0)) {
				rx_fifo_clear();
				prev_fifo_size = 0;
			}
		} // if (up_time >= algo_nextTime)
	} // while (1)
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// TEST SCRIPT - SIMULATION - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_simulation() {

	#include <_params_simulation.h>
	int step_sim = 0;
	int r_i, c_i;

	/////////////////////////////////////////////////////////////////////////////////////
	// Complete kinematics set-up:
	/////////////////////////////////////////////////////////////////////////////////////

	// Trajectory path - constraint matrices:
	nml_mat* A_con = nml_mat_new(N_CONSTR_TRAJ, N_COORD_EXT);
	nml_mat* b_con = nml_mat_new(N_CONSTR_TRAJ, 1);

	sig_exp    = 3.0/T_exp;
	dt_phi_ref = 2*PI/T_cycle;

	/////////////////////////////////////////////////////////////////////////////////////
	// ITM console output:
	/////////////////////////////////////////////////////////////////////////////////////

#if USE_ITM_OUT_SIM
	printf("%f\t%f\t%f\t%f\t%f\t%f\n",
		dt_k,
		T_cycle,
		T_exp,
		ax_x,
		ax_y,
		ax_ang);
#endif

	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////
	// USER CODE BEGIN WHILE - GAO
	/////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////

	do {

		/////////////////////////////////////////////////////////////////////////////////////
		// Apply delay:
		/////////////////////////////////////////////////////////////////////////////////////

		HAL_Delay(25);

		/////////////////////////////////////////////////////////////////////////////////////
		// Run simulation:
		/////////////////////////////////////////////////////////////////////////////////////

		t_ref  = dt_k*step_sim;
		phi_ref = dt_phi_ref*t_ref;

		//  Adjusted trajectory path dimensions:
		ax_x_adj = (1.0 - exp(-sig_exp*t_ref))*ax_x;
		ax_y_adj = (1.0 - exp(-sig_exp*t_ref))*ax_y;

		// Generate trajectory points:
		traj_ellipse_help(phi_ref, dt_phi_ref, p_ref, dt_p_ref, u_t_ref,
				A_con, b_con,
				ax_x_adj, ax_y_adj, ax_ang);

		/////////////////////////////////////////////////////////////////////////////////////
		// Simulation output:
		/////////////////////////////////////////////////////////////////////////////////////

		// Write feedback message:
		/*
		set_lowerlimb_exercise_feedback_info(
			p_ref[IDX_X], p_ref[IDX_Y],
			0, 0,
			dt_p_ref[IDX_X], dt_p_ref[IDX_Y],
			0, 0,
			0, 0,
			0, 0, // force Sensor
			0, 0, 0, // force commands
			0, 0, // reference positions
			0, 0, // reference velocities
			0, 0 // reference phase and frequency
		);
		*/

		// ITM console output:
		printf("%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t",
			step_sim,
			t_ref,
			phi_ref,
			dt_phi_ref,
			p_ref[IDX_X],
			p_ref[IDX_Y],
			dt_p_ref[IDX_X],
			dt_p_ref[IDX_Y]);

		for (r_i = 0; r_i < N_CONSTR_TRAJ; r_i++) {
			for (c_i = 0; c_i < N_COORD_EXT; c_i++)
				printf("%f\t", A_con->data[r_i][c_i]);
			printf("\n");
		}

		/////////////////////////////////////////////////////////////////////////////////////
		// Increase step counter:
		/////////////////////////////////////////////////////////////////////////////////////

		step_sim++;

	} while (t_ref <= T_MAX);
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// "SCRATCH" SCRIPT - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void
test_scratch() {

	/////////////////////////////////////////////////////////////////////////////
	// Test console:
	/////////////////////////////////////////////////////////////////////////////

	int i = 0;
	for (i = 1; i <= 10; i++) {
		printf("foo[%d]\n", i);
		HAL_Delay(100);
	}
	printf("\n");

	/////////////////////////////////////////////////////////////////////////////
	// Matrix multiplication:
	/////////////////////////////////////////////////////////////////////////////

	double a[] = {
			0.11, 0.12, 0.13,
			0.21, 0.22, 0.23 };
	double b[] = {
			1011, 1012,
			1021, 1022,
			1031, 1032 };

	nml_mat* A = nml_mat_from(2, 3, 6, a);
	nml_mat* B = nml_mat_from(3, 2, 6, b);

	// Multiply matrices
	nml_mat* C = nml_mat_dot(A, B);

	double CC[4];

	CC[0] = C->data[0][0];
	CC[1] = C->data[0][1];

	CC[2] = C->data[1][0];
	CC[3] = C->data[1][1];

	nml_mat* D = nml_mat_sqr(4);

	/////////////////////////////////////////////////////////////////////////////
	// Solve linear system:
	/////////////////////////////////////////////////////////////////////////////

	double a_data[] = {
		0.18, 0.60, 0.57, 0.96,
		0.41, 0.24, 0.99, 0.58,
		0.14, 0.30, 0.97, 0.66,
		0.51, 0.13, 0.19, 0.85 };

	double b_data[] = { 1.0, 2.0, 3.0, 4.0 };

	nml_mat* A_sys = nml_mat_from(4, 4, 16, a_data);
	nml_mat* b_sys = nml_mat_from(4, 1,  4, b_data);

	nml_mat_lup* A_LUP = nml_mat_lup_solve(A_sys);
	nml_mat*     x     = nml_ls_solve(A_LUP, b_sys);

	double xx[4];

	xx[0] = x->data[0][0];
	xx[1] = x->data[1][0];
	xx[2] = x->data[2][0];
	xx[3] = x->data[3][0];

	nml_mat* E = nml_mat_sqr(4);

	int r_i, c_i;
	for (r_i = 0; r_i < 2; r_i++) {
		printf("\n");
		for (c_i = 0; c_i < 2; c_i++) {
			printf("C[%d][%d] = %f\n", r_i, c_i, C->data[r_i][c_i]);
			HAL_Delay(100);
			// fflush(stdout);
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS - DEFINITIONS - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int
_write(int32_t file, uint8_t *ptr, int32_t len) {
	int i;
	for(i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}

/*
int
__io_putchar(int ch) {
	// Write character to ITM ch.0
	ITM_SendChar(ch);
	return(ch);
}
*/

void
cycle_haptic_buttons() {
	if (Read_Haptic_Button()) {
		Left_LED_function (Blue);
		Right_LED_function (Blue);
	}
	else {
		Left_LED_function (Red);
		Right_LED_function (Red);
	}
}


void
LED_sys_state_off() {
	Left_LED_function(Yellow);
	Right_LED_function(Yellow);
}

void
set_brakes_timed(uint64_t uptime, uint64_t* brakes_next_time) {
	if (uptime >= *brakes_next_time) {
		*brakes_next_time = uptime + 1; // 1kHz

		/////////////////////////////////////////////////////////////////////////////////////
		// Code to Engage & Disengage Brake of the Radial Axis
		/////////////////////////////////////////////////////////////////////////////////////

		set_l_brake_status(r_brakes(get_l_brake_cmd() && Read_Haptic_Button()));

		/////////////////////////////////////////////////////////////////////////////////////
		// Code to Engage & Disengage Brake of the Rotational Axis
		/////////////////////////////////////////////////////////////////////////////////////

		set_r_brake_status(p_brakes(get_r_brake_cmd() && Read_Haptic_Button()));
	}
}

uint8_t
set_LL_exercise_feedback_help(lowerlimb_mech_readings_t* mech_readings, lowerlimb_motors_settings_t* motor_settings) {

	float FORCE_END_MAGN = 0.0; // don't need to maintain this

	return set_lowerlimb_exercise_feedback_info(
		mech_readings->coord.x,
		mech_readings->coord.y,
		mech_readings->left.qei_count,
		mech_readings->right.qei_count,
		mech_readings->velocity.x,
		mech_readings->velocity.y,
		motor_settings->left.volt,
		motor_settings->right.volt,
		mech_readings->left.currsens_amps,
		mech_readings->right.currsens_amps,
		mech_readings->Xforce, // X-axis Force Sensor
		mech_readings->Yforce, // Y-axis Force Sensor
		motor_settings->force_end[IDX_X],
		motor_settings->force_end[IDX_Y],
		FORCE_END_MAGN,
		0, 0, 0, 0, 0, 0);
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS - GAO
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

/*
* @brief System Clock Configuration
* @retval None
*/

void
SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void
MX_ADC1_Init(void) {

  ADC_ChannelConfTypeDef sConfig;

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void
MX_ADC3_Init(void) {

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = ENABLE;
  hadc3.Init.NbrOfDiscConversion = 1;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void
MX_DAC_Init(void) {

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void
MX_SPI3_Init(void) {

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM2 init function */
static void
MX_TIM2_Init(void) {

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void
MX_TIM4_Init(void) {

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void
MX_TIM9_Init(void) {

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 179;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void
MX_USART3_UART_Init(void) {

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/

static void
MX_GPIO_Init(void) {

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, R_MD_DIO3_Pin|R_MD_DI2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R_RED_Pin|R_BLUE_Pin|R_GREEN_Pin|L_MD_DIO3_Pin
                          |L_MD_DI2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, L_BRAKE_16V_Pin|L_BRAKE_24V_Pin|R_BRAKE_16V_Pin|R_BRAKE_24V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_GREEN_Pin|L_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, L_RED_Pin|ETHERNET_SCSn_Pin|ETHERNET_RSTn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC5
                           PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF6
                           PF7 PF10 PF11 PF12
                           PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : R_MD_DIO3_Pin R_MD_DI2_Pin */
  GPIO_InitStruct.Pin = R_MD_DIO3_Pin|R_MD_DI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : R_RED_Pin R_BLUE_Pin R_GREEN_Pin L_MD_DIO3_Pin
                           L_MD_DI2_Pin */
  GPIO_InitStruct.Pin = R_RED_Pin|R_BLUE_Pin|R_GREEN_Pin|L_MD_DIO3_Pin
                          |L_MD_DI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : R_QEI_INDEX_Pin */
  GPIO_InitStruct.Pin = R_QEI_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(R_QEI_INDEX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA6 PA7 PA8
                           PA9 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 PB13
                           PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 PG6 PG7
                           PG8 PG9 PG10 PG11
                           PG12 PG13 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : L_BRAKE_16V_Pin L_BRAKE_24V_Pin R_BRAKE_16V_Pin R_BRAKE_24V_Pin */
  GPIO_InitStruct.Pin = L_BRAKE_16V_Pin|L_BRAKE_24V_Pin|R_BRAKE_16V_Pin|R_BRAKE_24V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : L_GREEN_Pin L_BLUE_Pin */
  GPIO_InitStruct.Pin = L_GREEN_Pin|L_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L_RED_Pin ETHERNET_RSTn_Pin */
  GPIO_InitStruct.Pin = L_RED_Pin|ETHERNET_RSTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD14 PD15
                           PD3 PD4 PD5 PD6
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : L_QEI_INDEX_Pin ETHERNET_INTn_Pin */
  GPIO_InitStruct.Pin = L_QEI_INDEX_Pin|ETHERNET_INTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ETHERNET_SCSn_Pin */
  GPIO_InitStruct.Pin = ETHERNET_SCSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ETHERNET_SCSn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SIG_EMGC_Pin */
  GPIO_InitStruct.Pin = SIG_EMGC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SIG_EMGC_GPIO_Port, &GPIO_InitStruct);
}

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */

void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


