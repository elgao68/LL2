
		///////////////////////////////////////////////////////////////////////////
		// OTHER TCP/IP COMMANDS:
		///////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_UNIX_CMD) { //set unix

			//pack UNIX
			ui64_tmp = 0;
			memcpy_msb(&ui64_tmp, &tcpRxData[payloadStart_index], sizeof(lowerlimb_sys_info.unix));

			//check if UNIX is valid
			if (!valid_app_status(set_unix_time(ui64_tmp), 0,
				&lowerlimb_sys_info.app_status, cmd_code, ERR_INVALID_UNIX, ERR_OFFSET))
					return lowerlimb_sys_info;

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == RESET_SYS_CMD) { //restart system

			HAL_Delay(100); //wait for msg to be transmitted
			HAL_NVIC_SystemReset(); //reset MCU

			send_OK_resp(cmd_code);
		}

		else if (cmd_code == READ_DEV_ID_CMD) { //read device ID
			// send resp message with device ID
			send_resp_msg(cmd_code, lowerlimb_sys_info.device_id, sizeof(lowerlimb_sys_info.device_id));
		}

		else if (cmd_code == READ_SYS_INFO_CMD) { //read system info
			// pack data
			send_lowerlimb_sys_info(&lowerlimb_sys_info, tmp_resp_msg, cmd_code);
		}

		else if (cmd_code == PAUSE_EXE_CMD) { //pause exercise

			// only can paused if exercise_state is in RUNNING mode:
			if (lowerlimb_sys_info.activity_state == EXERCISE &&
				lowerlimb_sys_info.exercise_state == RUNNING) {
					lowerlimb_sys_info.exercise_state = PAUSED;
					// send OK resp
					send_OK_resp(cmd_code);
			}
			else {
				// return error message
				send_error_msg(cmd_code, ERR_EXERCISE_NOT_RUNNING);
				lowerlimb_sys_info.app_status = ERR_EXERCISE_NOT_RUNNING + 3;
				return lowerlimb_sys_info;
			}
		}

		else if (cmd_code == TOGGLE_SAFETY_CMD) { //enable/disable safety features

			lowerlimb_sys_info.safetyOFF = tcpRxData[rx_payload_index];

			send_OK_resp(cmd_code);
		}

		////////////////////////////////////////////////////////////////////////////////
		// SET CONTROL GAINS:
		////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_CTRLPARAMS) {

			// Assign control parameter values:
			memcpy_msb(&fbgain, &tcpRxData[rx_payload_index], sizeof(fbgain));
			rx_payload_index += sizeof(fbgain);
			memcpy_msb(&ffgain, &tcpRxData[rx_payload_index], sizeof(ffgain));
			rx_payload_index += sizeof(ffgain);
			memcpy_msb(&compgain, &tcpRxData[rx_payload_index], sizeof(compgain));

			// Set control parameters
			configure_ctrl_params(fbgain, ffgain, compgain);

			send_OK_resp(cmd_code);
		}

		////////////////////////////////////////////////////////////////////////////////
		// SET PASSIVE TRAJECTORY CONTROL PARAMS:
		////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_PTRAJCTRL_CMD) {

			// Assign control parameter values:
			memcpy_msb(&cycle_period, &tcpRxData[rx_payload_index],
					   sizeof(cycle_period));
			rx_payload_index += sizeof(cycle_period);
			memcpy_msb(&exp_blending_time, &tcpRxData[rx_payload_index],
					   sizeof(exp_blending_time));
			rx_payload_index += sizeof(exp_blending_time);
			memcpy_msb(&semiaxis_x, &tcpRxData[rx_payload_index],
					   sizeof(semiaxis_x));
			rx_payload_index += sizeof(semiaxis_x);
			memcpy_msb(&semiaxis_y, &tcpRxData[rx_payload_index],
					   sizeof(semiaxis_y));
			rx_payload_index += sizeof(semiaxis_y);
			memcpy_msb(&rot_angle, &tcpRxData[rx_payload_index], sizeof(rot_angle));
			rx_payload_index += sizeof(rot_angle);
			cycle_dir = tcpRxData[rx_payload_index];

			#if SET_CTRL_PARAMETERS
				//Set passive trajectory control parameters:
				*traj_ctrl_params = set_traj_ctrl_params(cycle_period, exp_blending_time,
						semiaxis_x, semiaxis_y, rot_angle, cycle_dir);
			#endif

			lowerlimb_sys_info.exercise_state = RUNNING;

			send_OK_resp(cmd_code);
		}

		////////////////////////////////////////////////////////////////////////////////
		// SET ADMITTANCE CONTROL PARAMS:
		////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_ADMCTRL_CMD) { // rxPayload 65

			// Assign control parameter values:
			memcpy_msb(&inertia_x, &tcpRxData[rx_payload_index], sizeof(inertia_x));
			rx_payload_index += sizeof(inertia_x);
			memcpy_msb(&inertia_y, &tcpRxData[rx_payload_index], sizeof(inertia_y));
			rx_payload_index += sizeof(inertia_y);
			memcpy_msb(&damping, &tcpRxData[rx_payload_index], sizeof(damping));
			rx_payload_index += sizeof(damping);
			memcpy_msb(&stiffness, &tcpRxData[rx_payload_index], sizeof(stiffness));
			rx_payload_index += sizeof(stiffness);
			memcpy_msb(&p_eq_x, &tcpRxData[rx_payload_index], sizeof(p_eq_x));
			rx_payload_index += sizeof(p_eq_x);
			memcpy_msb(&p_eq_y, &tcpRxData[rx_payload_index], sizeof(p_eq_y));
			rx_payload_index += sizeof(p_eq_y);
			memcpy_msb(&Fx_offset, &tcpRxData[rx_payload_index], sizeof(Fx_offset));
			rx_payload_index += sizeof(Fx_offset);
			memcpy_msb(&Fy_offset, &tcpRxData[rx_payload_index], sizeof(Fy_offset));
			rx_payload_index += sizeof(Fy_offset);

			#if SET_CTRL_PARAMETERS
				//Set admittance control params:
				*admitt_model_params = set_admitt_model_params(inertia_x, inertia_y, damping,
						stiffness, p_eq_x, p_eq_y);
			#endif

			lowerlimb_sys_info.exercise_state = RUNNING;

			send_OK_resp(cmd_code);
		}

		////////////////////////////////////////////////////////////////////////////////
		// SET ACTIVE TRAJECTORY CONTROL PARAMS:
		////////////////////////////////////////////////////////////////////////////////

		else if (cmd_code == SET_TARG_PARAM_ATRAJCTRL_CMD) {

			// Assign control parameter values:
			memcpy_msb(&cycle_period, &tcpRxData[rx_payload_index],
					   sizeof(cycle_period));
			rx_payload_index += sizeof(cycle_period);
			memcpy_msb(&exp_blending_time, &tcpRxData[rx_payload_index],
					   sizeof(exp_blending_time));
			rx_payload_index += sizeof(exp_blending_time);
			memcpy_msb(&inertia_x, &tcpRxData[rx_payload_index], sizeof(inertia_x));
			rx_payload_index += sizeof(inertia_x);
			memcpy_msb(&inertia_y, &tcpRxData[rx_payload_index], sizeof(inertia_y));
			rx_payload_index += sizeof(inertia_y);
			memcpy_msb(&damping, &tcpRxData[rx_payload_index], sizeof(damping));
			rx_payload_index += sizeof(damping);
			memcpy_msb(&stiffness, &tcpRxData[rx_payload_index], sizeof(stiffness));
			rx_payload_index += sizeof(stiffness);
			memcpy_msb(&F_assist_resist, &tcpRxData[rx_payload_index],
					   sizeof(F_assist_resist));
			rx_payload_index += sizeof(F_assist_resist);

			#if SET_CTRL_PARAMETERS
				//Set passive trajectory control parameters:
				*traj_ctrl_params = set_traj_ctrl_params(cycle_period, exp_blending_time,
						semiaxis_x, semiaxis_y, rot_angle, cycle_dir);

				//Set admittance control params:
				*admitt_model_params = set_admitt_model_params(inertia_x, inertia_y, damping,
						stiffness, p_eq_x, p_eq_y);
			#endif

			lowerlimb_sys_info.exercise_state = RUNNING;

			send_OK_resp(cmd_code);
		}
