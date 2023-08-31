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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
//#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/adc_report.h>
//#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>

#include <uORB/topics/debug_array.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/input_rc.h>

#include "sticks.hpp"

extern "C" __EXPORT int sim_ctrl_mod_main(int argc, char *argv[]);



class sim_data_trafic
{
public:
	sim_data_trafic();
	~sim_data_trafic();

	static const uint MAX_SIZE = debug_array_s::ARRAY_SIZE;

	void send_vec(float out_vec[MAX_SIZE]);
	char fill_buffer(float in);
	char fill_buffer(float* in, uint size);
	void clear_buffer(void);

private:

	uint ind;
	float data[MAX_SIZE];
};


class SIM_CTRL_MOD : public ModuleBase<SIM_CTRL_MOD>, public ModuleParams
{
public:

	SIM_CTRL_MOD(int example_param, bool example_flag);

	virtual ~SIM_CTRL_MOD() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SIM_CTRL_MOD *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	debug_array_s sm_inbound{};
	debug_array_s sm_outbound{}; //new actuator data comes from simulink and is published here
	actuator_outputs_s act_output{}; //this is where actuator data has to be published

	void printf_debug_array(debug_array_s &array);
	void printf_actuator_output(actuator_outputs_s &array);

	void update_simulink_io(void);
	void update_simulink_inputs(void);
	void update_simulink_outputs(void);

	uORB::Publication<debug_array_s> 		_simulink_outbound_pub{ORB_ID(simulink_outbound)};
	uORB::Publication<debug_array_s>		_simulink_inbound_pub{ORB_ID(simulink_inbound)};
	uORB::Publication<actuator_outputs_s>		_actuator_outputs_sv_pub{ORB_ID(actuator_outputs_sv)};
	uORB::Publication<actuator_armed_s> 		_actuator_armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<manual_control_setpoint_s>	_manual_control_setpoint_pub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<manual_control_switches_s>	_manual_control_switches_pub{ORB_ID(manual_control_switches)};
	uORB::Publication<input_rc_s>			_input_rc_pub{ORB_ID(input_rc)};



	// Subscriptions
	uORB::Subscription		_parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription 		_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription 		_vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription		_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription		_airspeed_sub{ORB_ID(airspeed)};
	//uORB::Subscription		_battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription		_distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription		_actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription		_vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription		_adc_report_sub{ORB_ID(adc_report)};
	//uORB::Subscription		_obstacle_distance_sub{ORB_ID(obstacle_distance)};
	uORB::Subscription		_rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription		_rc_parameter_map_sub{ORB_ID(rc_parameter_map)};
	uORB::Subscription		_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription		_manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription		_simulink_outbound_sub{ORB_ID(simulink_outbound)};
	uORB::Subscription		_simulink_inbound_sub{ORB_ID(simulink_inbound)};
	uORB::Subscription 		_actuator_outputs_sv_sub{ORB_ID(actuator_outputs_sv)};

	vehicle_local_position_s local_pos{};
	vehicle_global_position_s global_pos{};
	vehicle_attitude_s att{};
	airspeed_s airspeed{};
	//battery_status_s batt;
	distance_sensor_s dist{};
	actuator_armed_s act_armed{};
	vehicle_odometry_s odom{};
	adc_report_s adc{};

	hrt_abstime	_boot_timestamp{0};

	//obstacle_distance_s obs;

	rc_channels_s rc_ch{};
	rc_parameter_map_s rc_map{};
	manual_control_setpoint_s man_setpoint{};
	manual_control_switches_s man_switches{};


	void publish_inbound_sim_data(void);
	sim_data_trafic simulink_inboud_data{};

	debug_array_s debug_topic{};

	bool check_ground_contact(void);
	bool update_distance_sensor(void);
	bool update_airspeed(void);
	bool update_adc(void);

	//All actuators are packed together and sent by the simulink model.
	static const int ACTUATOR_START_IND = 0; //this is the start index of actuator data in OUTBOUND data array
	static const int ACTUATOR_MAX_SIZE = actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; //maximum number of actuators (always send all)

	static const int CONTROL_VEC_SIZE = 18; //control vector size

	float control_vec[CONTROL_VEC_SIZE];

	bool update_control_inputs(float in_vec[CONTROL_VEC_SIZE]);
	bool check_armed(bool &armed, int input_src_opt);
	bool update_man_wing_angle(int input_source_opt, float& wing_cmd);
	bool update_sticks(int input_source_opt, sticks_ind stick, float& stick_val);

	void debug_loop(void);

	void test_fake_atuator_data(void);

	/**
	 * THIS IS WHERE YOU DEFINE NEW PARAMETRS
	 * example:
	 * DEFINE_PARAMETERS(
	 *	(ParamFloat<px4::params::SM_LQI_LG_SAT>) _param_sm_lqi_lg_sat)
	 *
	 * where:
	 *	DEFINE_PARAMETERS is macro (short function)
	 * 	ParamFloat defines the data type of the parameter (float in this case)
	 * 	SM_LQI_LG_SAT is the global name that other app will see (e.g. simulink or QGC). Make sure this is under 14 characters
	 *	_param_sm_lqi_lg_sat is the local variable (used in this app only).
	 *	They MUST BE UPPERCASE AND lowercase respectively (don't mix)
	 */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SM_EN_CAL>) _param_sm_en_cal,
		(ParamInt<px4::params::SM_OVERWRITE>) _param_sm_overwrite,
		(ParamInt<px4::params::SM_GC_OPT>) _param_gc_opt,
		(ParamInt<px4::params::SM_EN_HIL>) _param_en_hil,
		(ParamInt<px4::params::SM_MAV_STREAM>) _param_mav_stream,
		(ParamInt<px4::params::SM_CMD_OPT>) _param_cmd_opt,
		(ParamInt<px4::params::SM_WING_SRC>) _param_sm_wing_src
	)//MAKE SURE EVERY PARAMETER IS FOLLOWED BY "," AND LAST ONE DOES NOT HAVE ANYTHING
};

