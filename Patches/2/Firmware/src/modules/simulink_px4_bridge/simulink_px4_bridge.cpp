#include "simulink_px4_bridge.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <math.h>
#include <uORB/topics/parameter_update.h>
//#include "waypoints.hpp"

int SIMULINK_PX4_BRIDGE::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int SIMULINK_PX4_BRIDGE::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	return print_usage("unknown command");
}


int SIMULINK_PX4_BRIDGE::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("SIMULINK_PX4_BRIDGE",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT - 5,
				      1800,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SIMULINK_PX4_BRIDGE *SIMULINK_PX4_BRIDGE::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	// const char *file_string = nullptr;
	bool error_flag = false;

	// int myoptind = 1;
	// int ch;
	// const char *myoptarg = nullptr;

	// parse CLI arguments
	// while ((ch = px4_getopt(argc, argv, "p:f:", &myoptind, &myoptarg)) != EOF) {
	// 	switch (ch) {
	// 	case 'p':
	// 		example_param = (int)strtol(myoptarg, nullptr, 10);
	// 		PX4_INFO("p=%i",example_param);
	// 		break;

	// 	case 'f':
	// 		file_string = myoptarg;

	// 		PX4_INFO("f=%s",file_string);
	// 		break;

	// 	case '?':
	// 		error_flag = true;
	// 		break;

	// 	default:
	// 		PX4_WARN("unrecognized flag");
	// 		error_flag = true;
	// 		break;
	// 	}
	// }

	if (error_flag) {
		return nullptr;
	}

	SIMULINK_PX4_BRIDGE *instance = new SIMULINK_PX4_BRIDGE(example_param);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

SIMULINK_PX4_BRIDGE::SIMULINK_PX4_BRIDGE(int example_param)
	: ModuleParams(nullptr)
{
}

//#define DEBUG



void SIMULINK_PX4_BRIDGE::run()
{
	// initialize parameters
	parameters_update(true);

	_boot_timestamp = hrt_absolute_time();
	while (!should_exit()) {
		parameters_update(); // update parameters
		main_update_loop(); //update everything related to simulink

		px4_usleep(1E6/update_rate_HZ);// don't update too frequenty
	}
}

void SIMULINK_PX4_BRIDGE::main_update_loop(void)
{
	int32_t enable_fl = _param_sim_bridge_en.get();

	int value = 0;
	param_get(param_find("EXAMPLE_FLAG"),&value);


	if (enable_fl == 1)
	{
		#ifdef DEBUG
		PX4_INFO("Updating main loop");
		#endif

		if (update_all_px4_sensors()) publish_all_px4_sensors();
	}


}

bool SIMULINK_PX4_BRIDGE::update_all_px4_sensors(void)
{
	//poll new data if available:
	bool need_2_pub = false;

	if (_vehicle_local_position_sub.update(&local_pos)) need_2_pub = true;
	if (_vehicle_odometry_sub.update(&odom)) need_2_pub = true;
	if (_vehicle_global_position_sub.update(&global_pos)) need_2_pub = true;
	if (_vehicle_attitude_sub.update(&att)) need_2_pub = true;
	// if (update_control_inputs(control_vec)) need_2_pub = true;
	if (_distance_sensor_sub.update(&dist)) need_2_pub = true;
	if (_airspeed_sub.update(&airspeed)) need_2_pub = true;
	if (_vehicle_angular_velocity_sub.update(&v_angular_velocity)) need_2_pub = true;
	if (_vehicle_gps_position_sub.update(&veh_gps_position)) need_2_pub = true;
	if (_vehicle_acceleration_sub.update(&veh_acceleration)) need_2_pub = true;

	return need_2_pub;
}


void SIMULINK_PX4_BRIDGE::publish_all_px4_sensors(void)
{
	/////// populate main data fields: ///////////

	///## local position NED:
	//# Position in local NED frame
	sim_bridge_data.x = local_pos.x;				//# North position in NED earth-fixed frame, (metres)
	sim_bridge_data.y = local_pos.y;				//# East position in NED earth-fixed frame, (metres)
	sim_bridge_data.z = local_pos.z;				//# Down position (negative altitude) in NED earth-fixed frame, (metres)
	//# Velocity in NED frame
	sim_bridge_data.vx = local_pos.vx; 				//# North velocity in NED earth-fixed frame, (metres/sec)
	sim_bridge_data.vy = local_pos.vy;				//# East velocity in NED earth-fixed frame, (metres/sec)
	sim_bridge_data.vz = local_pos.vz;		//# Down velocity in NED earth-fixed frame, (metres/sec)
	sim_bridge_data.z_deriv	= local_pos.z_deriv;	//# Down position time derivative in NED earth-fixed frame, (metres/sec)
	//# Acceleration in NED frame
	sim_bridge_data.ax = local_pos.ax;       //# North velocity derivative in NED earth-fixed frame, (metres/sec^2)
	sim_bridge_data.ay = local_pos.ay;       //# East velocity derivative in NED earth-fixed frame, (metres/sec^2)
	sim_bridge_data.az = local_pos.az;       //# Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

	///## vehicle acceleration:
	sim_bridge_data.veh_ax = veh_acceleration.xyz[0];		//# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2
	sim_bridge_data.veh_ay = veh_acceleration.xyz[1];			//# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2
	sim_bridge_data.veh_az = veh_acceleration.xyz[2];			//# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2

	///## vehicle odometry:
	sim_bridge_data.odom_vz = odom.vz;			//# Down velocity

	///## vehicle angular velocity:
	sim_bridge_data.roll_rate = v_angular_velocity.xyz[0];	//# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s
	sim_bridge_data.pitch_rate = v_angular_velocity.xyz[1];	//# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s
	sim_bridge_data.yaw_rate = v_angular_velocity.xyz[2];	//# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s

	///## vehicle attitude:
	for (int i = 0; i < 4; i++) sim_bridge_data.q[i] = att.q[i];			    //# Quaternion rotation from the FRD body frame to the NED earth frame

	///## vehicle global position NED:
	sim_bridge_data.lat = global_pos.lat;			//# Latitude, (degrees)
	sim_bridge_data.lon = global_pos.lon;			//# Longitude, (degrees)
	sim_bridge_data.alt = global_pos.alt;			//# Altitude AMSL, (meters)
	sim_bridge_data.alt_ellipsoid = global_pos.alt_ellipsoid;		//# Altitude above ellipsoid, (meters)
	sim_bridge_data.terrain_alt = global_pos.terrain_alt;		//# Terrain altitude WGS84, (metres)
	sim_bridge_data.terrain_alt_valid = global_pos.terrain_alt_valid; //# Terrain altitude estimate is valid
	sim_bridge_data.dead_reckoning = global_pos.dead_reckoning;		//# True if this position is estimated through dead-reckoning

	///## vehicle gps position:
	sim_bridge_data.satellites_used = veh_gps_position.satellites_used;

	///## airspeed:
	sim_bridge_data.indicated_airspeed_m_s = airspeed.indicated_airspeed_m_s;		//# indicated airspeed in m/s
	sim_bridge_data.true_airspeed_m_s = airspeed.true_airspeed_m_s;		//# true filtered airspeed in m/s
	sim_bridge_data.air_temperature_celsius	= airspeed.air_temperature_celsius;	//# air temperature in degrees celsius, -1000 if unknown
	sim_bridge_data.airspeed_confidence = airspeed.confidence;		//# confidence value from 0 to 1 for this sensor

	///## distance sensor:
	sim_bridge_data.lidar_current_distance = dist.current_distance;	//# Current distance reading (in m)

	///////////////// finalize and publish the updated struct: ////////////
	sim_bridge_data.timestamp = hrt_absolute_time();
	_simulink_px4_bridge_data_pub.publish(sim_bridge_data);
}



void SIMULINK_PX4_BRIDGE::parameters_update(bool force)
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

int SIMULINK_PX4_BRIDGE::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided SIMULINK_PX4_BRIDGE module functionality.

This is a template for a SIMULINK_PX4_BRIDGE module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this SIMULINK_PX4_BRIDGE module.

### Examples
CLI usage example:
$ SIMULINK_PX4_BRIDGE start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("SIMULINK_PX4_BRIDGE", "custom");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int simulink_px4_bridge_main(int argc, char *argv[])
{
	return SIMULINK_PX4_BRIDGE::main(argc, argv);
}
