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
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_acceleration.h>

#include <uORB/topics/debug_array.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/input_rc.h>

#include <uORB/topics/simulink_px4_bridge_data.h>

extern "C" __EXPORT int simulink_px4_bridge_main(int argc, char *argv[]);


class SIMULINK_PX4_BRIDGE : public ModuleBase<SIMULINK_PX4_BRIDGE>, public ModuleParams
{
public:

	SIMULINK_PX4_BRIDGE(int example_param);

	virtual ~SIMULINK_PX4_BRIDGE() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SIMULINK_PX4_BRIDGE *instantiate(int argc, char *argv[]);

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

	uint update_rate_HZ = 100;
	void main_update_loop(void);

	hrt_abstime	_boot_timestamp{0};

	bool update_all_px4_sensors(void);
	void publish_all_px4_sensors(void);

	// Publications
	uORB::Publication<simulink_px4_bridge_data_s>	_simulink_px4_bridge_data_pub{ORB_ID(simulink_px4_bridge_data)};


	// Subscriptions
	uORB::Subscription		_parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription 		_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription 		_vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription		_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription		_airspeed_sub{ORB_ID(airspeed)};
	//uORB::Subscription		_battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription		_distance_sensor_sub{ORB_ID(distance_sensor)};
	// uORB::Subscription		_actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription		_vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	// uORB::Subscription		_adc_report_sub{ORB_ID(adc_report)};
	//uORB::Subscription		_obstacle_distance_sub{ORB_ID(obstacle_distance)};
	// uORB::Subscription		_rc_channels_sub{ORB_ID(rc_channels)};
	// uORB::Subscription		_rc_parameter_map_sub{ORB_ID(rc_parameter_map)};
	// uORB::Subscription		_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	// uORB::Subscription		_manual_control_switches_sub{ORB_ID(manual_control_switches)};
	// uORB::Subscription 		_actuator_outputs_sv_sub{ORB_ID(actuator_outputs_sv)};
	//uORB::Subscription 		_vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
	uORB::Subscription     		_vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription		_vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription		_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	//data structures for storing most recent data
	vehicle_local_position_s local_pos{};
	vehicle_global_position_s global_pos{};
	vehicle_attitude_s att{};
	airspeed_s airspeed{};
	//battery_status_s batt;
	distance_sensor_s dist{};
	actuator_armed_s act_armed{};
	// actuator_armed_s act_armed_px4{};
	vehicle_odometry_s odom{};
	// adc_report_s adc{};
	vehicle_angular_velocity_s v_angular_velocity{};
	vehicle_gps_position_s veh_gps_position{};
	vehicle_acceleration_s veh_acceleration{};

	simulink_px4_bridge_data_s sim_bridge_data{};

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
		(ParamInt<px4::params::SIM_BRIDGE_EN>) _param_sim_bridge_en
	)//MAKE SURE EVERY PARAMETER IS FOLLOWED BY "," AND LAST ONE DOES NOT HAVE ANYTHING
};

