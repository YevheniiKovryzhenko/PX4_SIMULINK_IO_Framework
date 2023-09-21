/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "sim_ctrl_mod.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <math.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_rc_input.h>

sim_data_trafic::sim_data_trafic()
{
	ind = 0;
	for (uint i = 0; i < MAX_SIZE; i++)
	{
		data[i] = 0.f;
	}
}

sim_data_trafic::~sim_data_trafic()
{
}

void sim_data_trafic::send_vec(float out_vec[MAX_SIZE])
{
	for (uint i = 0; i < MAX_SIZE; i++) out_vec[i] = data[i];
	clear_buffer();

}

char sim_data_trafic::fill_buffer(float* in, uint size)
{
	for (uint i = 0; i < size; i++)
	{
		fill_buffer(in[i]);
	}
	return 0;
}

char sim_data_trafic::fill_buffer(float in)
{
	if (ind < MAX_SIZE) {
		data[ind] = in;
		ind++;
		return 0;
	}
	else
	{
		clear_buffer();
#ifdef DEBUG
PX4_INFO("ERROR: ran out of space in buffer, check data size\n");
#endif
		return -1;
	}

}
void sim_data_trafic::clear_buffer(void)
{
	for (uint i = 0; i < MAX_SIZE; i++) data[i] = 0.f;
	ind = 0;

}

int SIM_CTRL_MOD::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int SIM_CTRL_MOD::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	*/

	return print_usage("unknown command");
}


int SIM_CTRL_MOD::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sim_ctrl_mod",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SIM_CTRL_MOD *SIM_CTRL_MOD::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	SIM_CTRL_MOD *instance = new SIM_CTRL_MOD(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

SIM_CTRL_MOD::SIM_CTRL_MOD(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
	for (int i = 0; i < CONTROL_VEC_SIZE; i++)
	{
		control_vec[i] = 0.0;
	}
}

//#define DEBUG

void SIM_CTRL_MOD::debug_loop(void)
{
	//actuator_outputs_s act_out{};
	//if (_simulink_inbound_sub.update(&sm_inbound)) printf_debug_array(sm_inbound);
	//if (_actuator_outputs_sv_sub.update(&act_out)) printf_actuator_output(act_out);
	/*
	if (_simulink_outbound_sub.updated())
	{
		PX4_INFO("received outbound data\n");
	}
	*/

	//test_fake_atuator_data();
}

void SIM_CTRL_MOD::run()
{
	// initialize parameters
	parameters_update(true);

	_boot_timestamp = hrt_absolute_time();
	while (!should_exit()) {
		parameters_update(); // update parameters
		update_simulink_io(); //update everything related to simulink

		px4_usleep(1000);// don't update too frequenty
	}
}

void SIM_CTRL_MOD::update_simulink_io(void)
{
	update_simulink_inputs();

	update_simulink_outputs();

	#ifdef DEBUG
	debug_loop();
	#endif
}

void SIM_CTRL_MOD::update_simulink_inputs(void)
{
	/*
	float SM_IDLE_TH_val = 0;
	param_get(param_find("SM_MASS"),&SM_IDLE_TH_val);
	printf("updating main loop:\t");
	printf("SM_MASS = %f\n",(double)SM_IDLE_TH_val);
	*/

	// we will later do some extra parsing and safety in here before publishing any data
	publish_inbound_sim_data();

}

void SIM_CTRL_MOD::update_simulink_outputs(void)
{
	bool need2update_actuators = false;
	int32_t SM_EN_DIRECT = 0;
	param_get(param_find("SM_EN_DIRECT"), &SM_EN_DIRECT);

	if (SM_EN_DIRECT == 1) //use user-set manual map
	{
		debug_array_s inbound_data;
		int input_source_opt = _param_cmd_opt.get();
		if (input_source_opt == 2) inbound_data = sm_inbound;
		else inbound_data = debug_topic;

		act_output.timestamp = hrt_absolute_time();
		act_output.noutputs = ACTUATOR_MAX_SIZE;
		for(uint8_t i = 0; i < ACTUATOR_MAX_SIZE; i++ )
		{
			act_output.output[i] = 1000.f * (inbound_data.data[get_output_ind(i)] + 1.f) / 2.f; //[-1 1] -> [0 1000]
		}

		need2update_actuators = true;

	}
	else //use simulink output
	{
		//if simulink has published new outputs, we have to grab those and distribute to proper places in PX4
		if (_simulink_outbound_sub.update(&sm_outbound))
		{
			act_output.timestamp = hrt_absolute_time();
			act_output.noutputs = ACTUATOR_MAX_SIZE;
			for(int i = 0; i < ACTUATOR_MAX_SIZE; i++ )
			{
				act_output.output[i] = sm_outbound.data[i];
			}

			need2update_actuators = true;
		}
	}



	//we can do some safety checking here first, before publishing the values

	if(need2update_actuators) _actuator_outputs_sv_pub.publish(act_output); //the rest of PX4 needs to know new actuator commands
}

bool SIM_CTRL_MOD::update_sticks(int input_source_opt, sticks_ind stick, float& stick_val)
{
	switch (input_source_opt)
		{
		case 1: // RC_IN
		{
			CASE_RC_IN:
			_rc_channels_sub.update(&rc_ch);

			//check every stick so we are sure rc is valid, otherwise quit
			if (sticks_ind2rc_channels(stick, rc_ch, stick_val) == 0)
			{
				switch (stick)
				{
				case THROTTLE: //special case
					stick_val = stick_val * 2.f - 1.f; // [0 1] -> [-1 1]
					return true;

				default:
					return true;
				}
			}
			else return false;
		}

		case 2: //INBOUND_MSG
		{
			CASE_INBOUND_MSG:
			_simulink_inbound_sub.update(&sm_inbound);
			stick_val = sm_inbound.data[get_input_ind(stick)];
			return true;//assume everything was already sent correctly
		}

		case 3: //INBOUND_CONTROL_MSG
		{
			CASE_INBOUND_CONTROL_MSG:
			_simulink_inbound_ctrl_sub.update(&sm_inbound_ctrl);
			stick_val = sm_inbound_ctrl.data[get_input_ind(stick)];
			return true;//assume everything was already sent correctly
		}

		case 4: //use AUX5 to decide (MANUAL_CONTROL_SETPOINT / INBOUND_CONTROL_MSG)
		{
			_rc_channels_sub.update(&rc_ch);

			//check every stick so we are sure rc is valid, otherwise quit
			static float aux5 = -1.f;
			sticks_ind2rc_channels(sticks_ind::AUX5, rc_ch, aux5);
			if (aux5 > 0.0f) goto CASE_INBOUND_CONTROL_MSG;
			else goto CASE_MANUAL_CONTROL_SETPOINT;
		}

		case 5: //use AUX5 to decide (RC_IN / INBOUND_CONTROL_MSG)
		{
			_rc_channels_sub.update(&rc_ch);

			//check every stick so we are sure rc is valid, otherwise quit
			static float aux5 = -1.f;
			sticks_ind2rc_channels(sticks_ind::AUX5, rc_ch, aux5);
			if (aux5 > 0.0f) goto CASE_INBOUND_CONTROL_MSG;
			else goto CASE_RC_IN;
		}

		case 6: //use AUX5 to decide (INBOUND_MSG / INBOUND_CONTROL_MSG)
		{
			_rc_channels_sub.update(&rc_ch);

			//check every stick so we are sure rc is valid, otherwise quit
			static float aux5 = -1.f;
			sticks_ind2rc_channels(sticks_ind::AUX5, rc_ch, aux5);
			if (aux5 > 0.0f) goto CASE_INBOUND_CONTROL_MSG;
			else goto CASE_INBOUND_MSG;
		}

		default: //MANUAL_CONTROL_SETPOINT
		{
			CASE_MANUAL_CONTROL_SETPOINT:
			_manual_control_setpoint_sub.update(&man_setpoint);
			_manual_control_switches_sub.update(&man_switches);

			switch (stick)
			{
			case THROTTLE: //special case
				stick_val = stick_val * 2.f - 1.f; // [0 1] -> [-1 1]
				return true;
			case MODE:
				stick_val = man_switches.mode_slot; //special case
				return true;

			default:
				if (sticks_ind2manual_control(stick, man_setpoint, stick_val) == 0)
				{
					return true;
				}
				else return false;
			}
		}
		}
	return false;
}

bool SIM_CTRL_MOD::update_man_wing_angle(int input_source_opt, float& wing_cmd)
{
	int32_t wing_opt = _param_sm_wing_src.get();
	bool need2update = false;
	static float wing_cmd_old = -1.f;

	switch (wing_opt)
	{
	case 1: //always 1
		wing_cmd = 1.f;
		break;

	case 2: //MANUAL use CMD_SRC
		if (update_sticks(input_source_opt, sticks_ind::AUX6, wing_cmd))
		{
			wing_cmd_old = wing_cmd;
			return true;
		}
		else
		{
			wing_cmd = wing_cmd_old;
			return false;
		}
	case 3: //AUTO
		if (update_sticks(input_source_opt, sticks_ind::AUX6, wing_cmd))
		{
			wing_cmd_old = wing_cmd;
			return true;
		}
		else
		{
			wing_cmd = wing_cmd_old;
			return false;
		}
	default: //always -1
		wing_cmd = -1.f;
		break;
	}
	need2update = (need2update || fabsf(wing_cmd - wing_cmd_old) > 1.f / 1000000.f);
	wing_cmd_old = wing_cmd; //keep track of the old command
	return need2update;
}

bool SIM_CTRL_MOD::update_distance_sensor(void)
{
	return _distance_sensor_sub.update(&dist);
}

bool SIM_CTRL_MOD::update_airspeed(void)
{
	return _airspeed_sub.update(&airspeed);
}

bool SIM_CTRL_MOD::update_adc(void)
{
	return _adc_report_sub.update(&adc);
}

bool SIM_CTRL_MOD::check_ground_contact(void) // this is a quick work-around the weird land-detector logic
{
	int32_t gc_case = _param_gc_opt.get();


	static int64_t time_pressed_gc = hrt_absolute_time();
	static bool was_pressed = false;

	static bool old_value = false;


	switch (gc_case)
	{
		case 1:
			old_value = true;
			was_pressed = false;
			break;
		case 2:
		{
			//int32_t use_lidar = 0;
			//param_get(param_find("SENS_EN_SF1XX"), &use_lidar);
			//if (use_lidar > 6) { //this is not needed anymore
				update_distance_sensor();
				float ekf2_min_rng = 0.f;
				param_get(param_find("EKF2_MIN_RNG"), &ekf2_min_rng);
				old_value = dist.current_distance < ekf2_min_rng;
			//}
			was_pressed = false;
			break;
		}
		case 3:
		{
			//adc_report_s adc;
			if (update_adc())
			{
				if (adc.resolution == 0) break;

				static const int ADC_GC_INDEX = 4;
				float val = static_cast<float>(adc.raw_data[ADC_GC_INDEX]) * static_cast<float>(adc.raw_data[ADC_GC_INDEX]) / static_cast<float> (adc.resolution);

				if(val < 1.f)
				{
					if (!was_pressed)
					{
						time_pressed_gc = hrt_absolute_time();
						old_value = false;
						was_pressed = true;
					}
					else
					{
						float landing_trig_time = 0.f;
						param_get(param_find("LNDMC_TRIG_TIME"), &landing_trig_time);

						float dt_sec = static_cast<float>(hrt_absolute_time() - time_pressed_gc) / 1000000.f;
						old_value = dt_sec > landing_trig_time;
					}

				}
				else was_pressed = false;
			}
			break;
		}
		default:
			old_value = false;
			was_pressed = false;
			break;
	}
	return old_value;

}

bool SIM_CTRL_MOD::check_armed(bool &armed, int input_src_opt)
{
	actuator_armed_s act_armed_px4;
	bool commander_updated_armed_state = _actuator_armed_sub.update(&act_armed_px4);

	//if (commander_updated_armed_state) printf("%4d, %4d, %4d\n", act_armed_px4.armed, act_armed_px4.prearmed, act_armed_px4.ready_to_arm);

	int32_t arm_src_opt = _param_sm_overwrite.get();

	if ((hrt_elapsed_time(&_boot_timestamp) > 5000000))
	{
		switch (arm_src_opt)
		{
		case 1: //always armed
			armed = true;
			break;

		case 2: //always disarmed
			armed = false;
			break;

		case 3: //use rc input message
		{
			float tmp_armed = static_cast<float>(armed);
			update_sticks(1, sticks_ind::ARMED, tmp_armed);
			armed = tmp_armed > 0.1f;
			break;
		}

		case 4: //use simulink inbound message
		{
			float tmp_armed = static_cast<float>(armed);
			update_sticks(2, sticks_ind::ARMED, tmp_armed);
			armed = tmp_armed > 0.1f;
			break;
		}

                case 5: //use simulink inbound control message
		{
			float tmp_armed = static_cast<float>(armed);
			update_sticks(3, sticks_ind::ARMED, tmp_armed);
			armed = tmp_armed > 0.1f;
			break;
		}

		case 6: //use default px4 logic
			PX4_INTERNAL_ARM:
			if (commander_updated_armed_state)
			{
				act_armed.timestamp = act_armed_px4.timestamp;
				act_armed.armed = act_armed_px4.armed;
				act_armed.force_failsafe = act_armed_px4.force_failsafe;
				act_armed.in_esc_calibration_mode = act_armed_px4.in_esc_calibration_mode;
				act_armed.lockdown = act_armed_px4.lockdown;
				act_armed.manual_lockdown = act_armed_px4.manual_lockdown;
				act_armed.prearmed = act_armed_px4.prearmed;
				act_armed.ready_to_arm = act_armed_px4.ready_to_arm;
				act_armed.soft_stop = act_armed_px4.soft_stop;
				armed = act_armed_px4.armed;
				return true;
			}
			else
			{
				armed = act_armed.armed;
				return false;
			}
		default: //use INPUT_SRC_OPT
			switch (input_src_opt)
			{
			case 0:
				goto PX4_INTERNAL_ARM;
			default:
			{
				float tmp_armed = static_cast<float>(armed);
				update_sticks(input_src_opt, sticks_ind::ARMED, tmp_armed);
				armed = tmp_armed > 0.1f;
				break;
			}
			}

			break;


		}
		if (armed != act_armed.armed || (commander_updated_armed_state && act_armed_px4.armed != armed))
		{
			act_armed.timestamp = hrt_absolute_time();
			act_armed.ready_to_arm = true;
			act_armed.prearmed = true;
			act_armed.armed = armed;
			act_armed.force_failsafe = false;
			act_armed.lockdown =false;
			act_armed.manual_lockdown = false;
			_actuator_armed_pub.publish(act_armed);
			return true;
		}
		else return false;
	}
	return false;
}

enum control_level
{
	CALIBRATION = 1,
	MODE1 = 2,
	MODE2 = 3,
	MODE3 = 4
};

void SIM_CTRL_MOD::printf_debug_array(debug_array_s &array)
{
	static uint64_t old_time = array.timestamp;
	double update_rate_Hz = 1000000.0 / (static_cast<double>(array.timestamp - old_time));
	printf("UpdateRate: %3.1f; HEADER: Name: %10s, ID: %5i, Timestamp %" PRIu64 , update_rate_Hz, array.name, array.id, array.timestamp);
	for (int i = 0; i < 58; i++)
	{
		printf(", %3.1f",static_cast<double>(array.data[i]));
	}
	printf("\33[2K\r");
	old_time = array.timestamp;
}

void SIM_CTRL_MOD::printf_actuator_output(actuator_outputs_s &outputs)
{
	static uint64_t old_time = outputs.timestamp;
	double update_rate_Hz = 1000000.0 / (static_cast<double>(outputs.timestamp - old_time));
	printf("UpdateRate: %3.1f; HEADER: N: %5i, Timestamp %" PRIu64 , update_rate_Hz, outputs.noutputs, outputs.timestamp);
	for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++)
	{
		printf(", %3.1f",static_cast<double>(outputs.output[i]));
	}
	printf("\33[2K\r");
	old_time = outputs.timestamp;
	return;
}

void SIM_CTRL_MOD::test_fake_atuator_data(void)
{
	actuator_outputs_s sv_out{};

	//sv_out.noutputs = 16;
	sv_out.timestamp = hrt_absolute_time();
	for (int i = 0; i < 16; i++)
	{
		sv_out.output[i] = static_cast<float>(i) * 50.f;
	}

	_actuator_outputs_sv_pub.publish(sv_out);
}



bool SIM_CTRL_MOD::update_control_inputs(float in_vec[CONTROL_VEC_SIZE])
{
	bool need_update = false;
	int input_source_opt = _param_cmd_opt.get();

	//Control vector elements:
	static float roll = 0.f;		//[-1 1]
	static float pitch = 0.f; 		//[-1 1]
	static float yaw = 0.f;			//[-1 1]
	static float throttle = -1.f;		//[-1 1]
	static float mode_stick = -1.f; 	//[-1 1] actual mode stick position
	static float aux1 = -1.f;		//[-1 1]
	static float aux2 = -1.f;		//[-1 1]
	static float aux3 = -1.f;		//[-1 1]
	static float aux4 = -1.f;		//[-1 1]
	static float aux5 = -1.f;		//[-1 1]
	static float aux6 = -1.f;		//[-1 1]
	static float extr1 = 0.f;
	static float extr2 = 0.f;
	static float extr3 = 0.f;
	static float extr4 = 0.f;
	static float extr5 = 0.f;
	static float extr6 = 0.f;


	static control_level mode_ch = MODE1;		//[1 4] mode that corresponds to the stick position

	int32_t en_calibration = _param_sm_en_cal.get();

	if (update_sticks(input_source_opt, sticks_ind::ROLL, roll)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::PITCH, pitch)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::YAW, yaw)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::THROTTLE, throttle)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::AUX1, aux1)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::AUX2, aux2)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::AUX3, aux3)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::AUX4, aux4)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::AUX5, aux5)) need_update = true;
	if (update_man_wing_angle(input_source_opt, aux6)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::EXTR1, extr1)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::EXTR2, extr2)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::EXTR3, extr3)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::EXTR4, extr4)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::EXTR5, extr5)) need_update = true;
	if (update_sticks(input_source_opt, sticks_ind::EXTR6, extr6)) need_update = true;

	if (update_sticks(input_source_opt, sticks_ind::MODE, mode_stick)) need_update = true;
	bool armed_switch = false;
	if (check_armed(armed_switch, input_source_opt)) need_update = true;

	if (en_calibration == 1)
	{
		mode_ch = CALIBRATION;
	}
	else
	{
		if (mode_stick > 0.7f) mode_ch = MODE3;
		else if(mode_stick < -0.7f) mode_ch = MODE1;
		else mode_ch = MODE2; //must always assign some default
	}

	in_vec[get_input_ind(sticks_ind::ROLL)] = roll;
	in_vec[get_input_ind(sticks_ind::PITCH)] = pitch;
	in_vec[get_input_ind(sticks_ind::YAW)] = yaw;
	in_vec[get_input_ind(sticks_ind::THROTTLE)] = throttle;
	in_vec[get_input_ind(sticks_ind::ARMED)] = static_cast<float>(armed_switch);
	in_vec[get_input_ind(sticks_ind::MODE)] = static_cast<float>(mode_ch);
	in_vec[get_input_ind(sticks_ind::AUX1)] = aux1;
	in_vec[get_input_ind(sticks_ind::AUX2)] = aux2;
	in_vec[get_input_ind(sticks_ind::AUX3)] = aux3;
	in_vec[get_input_ind(sticks_ind::AUX4)] = aux4;
	in_vec[get_input_ind(sticks_ind::AUX5)] = aux5;
	in_vec[get_input_ind(sticks_ind::AUX6)] = aux6;
	in_vec[get_input_ind(sticks_ind::EXTR1)] = extr1;
	in_vec[get_input_ind(sticks_ind::EXTR2)] = extr2;
	in_vec[get_input_ind(sticks_ind::EXTR3)] = extr3;
	in_vec[get_input_ind(sticks_ind::EXTR4)] = extr4;
	in_vec[get_input_ind(sticks_ind::EXTR5)] = extr5;
	in_vec[get_input_ind(sticks_ind::EXTR6)] = extr6;


	/*
	if (publish_new_manual_control_setpoint) //let the rest of PX4 know we have control input
	{

		//PX4_INFO("Updating manual control from INBOUND");
		//PARAM_DEFINE_INT32(COM_RC_IN_MODE, 0);

		int32_t com_rc_in_mode = 0;
		param_get(param_find("COM_RC_IN_MODE"), &com_rc_in_mode);


		if (com_rc_in_mode == 1)
		{
			//PX4_INFO("Updating manual control setpoint directly from INBOUND");
			man_setpoint.timestamp = hrt_absolute_time();
			man_setpoint.y = roll;
			man_setpoint.x = pitch;
			man_setpoint.r = yaw;
			man_setpoint.z = throttle;
			man_setpoint.aux6 = manual_wing_ch;
			man_setpoint.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;

			_manual_control_setpoint_pub.publish(man_setpoint);

			man_switches.timestamp = hrt_absolute_time();
			man_switches.arm_switch = armed_switch;
			man_switches.kill_switch = !armed_switch;
			man_switches.mode_switch = mode_stick;

			_manual_control_switches_pub.publish(man_switches);
		}
		else
		{
			//PX4_INFO("Updating virtual rc_input from INBOUND");
			input_rc_s rc{};
			rc.timestamp = hrt_absolute_time();
			rc.timestamp_last_signal = rc.timestamp;

			rc.channel_count = 8;
			rc.rc_failsafe = false;
			rc.rc_lost = false;
			rc.rc_lost_frame_count = 0;
			rc.rc_total_frame_count = 1;
			rc.rc_ppm_frame_length = 0;
			rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
			rc.rssi = RC_INPUT_RSSI_MAX;

			rc.values[0] = static_cast<uint16_t>(roll * 500.f) + 1500;	// roll
			rc.values[1] = static_cast<uint16_t>(pitch * 500.f) + 1500;	// pitch
			rc.values[2] = static_cast<uint16_t>(yaw * 500.f) + 1500;	// yaw
			rc.values[3] = static_cast<uint16_t>(throttle * 1000.f) + 1000;	// throttle

			// decode all switches which fit into the channel mask
			unsigned max_switch = 8;
			unsigned max_channels = (sizeof(rc.values) / sizeof(rc.values[0]));

			if (max_switch > (max_channels - 4)) {
				max_switch = (max_channels - 4);
			}
			rc.values[4] = static_cast<uint16_t>(manual_wing_ch * 500.f) + 1500;
			rc.values[5] = static_cast<uint16_t>(armed_switch)*1000 + 1000;
			rc.values[6] = static_cast<uint16_t>(mode_stick)*1000 + 1000;

			_input_rc_pub.publish(rc);
		}
	}
	*/

	return need_update;
}

void
SIM_CTRL_MOD::publish_inbound_sim_data(void)
{
	//poll new data if available:
	bool need_2_pub = false;

	if (_vehicle_local_position_sub.update(&local_pos)) need_2_pub = true;
	if (_vehicle_odometry_sub.update(&odom)) need_2_pub = true;
	if (_vehicle_global_position_sub.update(&global_pos)) need_2_pub = true;
	if (_vehicle_attitude_sub.update(&att)) need_2_pub = true;
	if (update_control_inputs(control_vec)) need_2_pub = true;
	if (update_distance_sensor()) need_2_pub = true;
	if (update_airspeed()) need_2_pub = true;



	//publish new data if needed:
	if (need_2_pub)
	{
		simulink_inboud_data.fill_buffer(control_vec, CONTROL_VEC_SIZE);

		simulink_inboud_data.fill_buffer(local_pos.vx); //1
		simulink_inboud_data.fill_buffer(local_pos.vy); //2
		simulink_inboud_data.fill_buffer(local_pos.vz); //3

		simulink_inboud_data.fill_buffer(odom.rollspeed); //4
		simulink_inboud_data.fill_buffer(odom.pitchspeed); //5
		simulink_inboud_data.fill_buffer(odom.yawspeed); //6

		simulink_inboud_data.fill_buffer(odom.q, 4); //7 8 9 10

		simulink_inboud_data.fill_buffer(global_pos.lat); // 11
		simulink_inboud_data.fill_buffer(global_pos.lon); // 12
		simulink_inboud_data.fill_buffer(global_pos.alt); // 13
		simulink_inboud_data.fill_buffer(global_pos.terrain_alt); //14

		simulink_inboud_data.fill_buffer(local_pos.ax); //15
		simulink_inboud_data.fill_buffer(local_pos.ay); //16
		simulink_inboud_data.fill_buffer(local_pos.az); //17

		simulink_inboud_data.fill_buffer(local_pos.x); //18
		simulink_inboud_data.fill_buffer(local_pos.y); //19
		simulink_inboud_data.fill_buffer(local_pos.z); //20

		simulink_inboud_data.fill_buffer(local_pos.z_deriv); //21

		simulink_inboud_data.fill_buffer(static_cast<float> (check_ground_contact())); //22

		simulink_inboud_data.fill_buffer(airspeed.true_airspeed_m_s); //23
		simulink_inboud_data.fill_buffer(airspeed.indicated_airspeed_m_s); //24
		simulink_inboud_data.fill_buffer(dist.current_distance); //25

		//publish new data:
		debug_topic.timestamp = hrt_absolute_time();
		debug_topic.id = debug_array_s::SIMULINK_INBOUND_ID;
		char message_name[10] = "inbound";
		memcpy(debug_topic.name, message_name, sizeof(message_name));
		debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

		simulink_inboud_data.send_vec(debug_topic.data);

		if (_param_en_hil.get() == 0) _simulink_inbound_pub.publish(debug_topic); //if HIL/SITL is enabled, assume this data was already published

		//_simulink_outbound_pub.publish(debug_topic);
	}

}


void SIM_CTRL_MOD::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int SIM_CTRL_MOD::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided sim_ctrl_mod module functionality.

This is a template for a sim_ctrl_mod module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this sim_ctrl_mod module.

### Examples
CLI usage example:
$ sim_ctrl_mod start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sim_ctrl_mod", "custom");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int sim_ctrl_mod_main(int argc, char *argv[])
{
	return SIM_CTRL_MOD::main(argc, argv);
}
