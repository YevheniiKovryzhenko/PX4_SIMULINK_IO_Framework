uint64 timestamp	# time since system start (microseconds)

## local position NED:
# Position in local NED frame
float32 x				# North position in NED earth-fixed frame, (metres)
float32 y				# East position in NED earth-fixed frame, (metres)
float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)
# Velocity in NED frame
float32 vx 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 vy				# East velocity in NED earth-fixed frame, (metres/sec)
float32 vz				# Down velocity in NED earth-fixed frame, (metres/sec)
float32 z_deriv				# Down position time derivative in NED earth-fixed frame, (metres/sec)
# Acceleration in NED frame
float32 ax        # North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 ay        # East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 az        # Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

## vehicle acceleration:
float32 veh_ax			# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2
float32 veh_ay			# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2
float32 veh_az			# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2


## vehicle odometry:
float32 odom_vz			# Down velocity

## vehicle angular velocity:
float32 roll_rate	# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s
float32 pitch_rate	# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s
float32 yaw_rate	# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s

## vehicle attitude:
float32[4] q			    # Quaternion rotation from the FRD body frame to the NED earth frame

## vehicle global position NED:
float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)
float32 alt_ellipsoid		# Altitude above ellipsoid, (meters)
float32 terrain_alt		# Terrain altitude WGS84, (metres)
bool terrain_alt_valid		# Terrain altitude estimate is valid
bool dead_reckoning		# True if this position is estimated through dead-reckoning

## vehicle gps position:
uint8 satellites_used		# Number of satellites used

## airspeed:
float32 indicated_airspeed_m_s		# indicated airspeed in m/s
float32 true_airspeed_m_s		# true filtered airspeed in m/s
float32 air_temperature_celsius		# air temperature in degrees celsius, -1000 if unknown
float32 airspeed_confidence		# confidence value from 0 to 1 for this sensor

## distance sensor:
float32 lidar_current_distance	# Current distance reading (in m)
