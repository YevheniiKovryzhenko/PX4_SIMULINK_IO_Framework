#include <matrix/math.hpp>
#include "trajectory.hpp"

#define DATATYPE_TRAJ float //shortcut for testing double vs float
static const int XYZ_OFFSET_START_IND = 18 + 17; //control vector size + number of states before xyz
static const int XYZ_VEL_OFFSET_START_IND = 18 + 0; //control vector size + number of states before xyz velocity
static const int QUAT_OFFSET_START_IND = 18 + 6; //control vector size + number of states before quat
static const int XYZ_ACC_OFFSET_START_IND = 18 + 14; //control vector size + number of states before accel
static const int PQR_OFFSET_START_IND = 18 + 3; //control vector size + number of states before pqr

double get_dt_s_hrt(hrt_abstime &time_stamp)
{
	return static_cast<double>(hrt_elapsed_time(&time_stamp))*1.0E-6;
}

template <typename Type, size_t n_coeffs, size_t n_dofs, size_t n_int>
void assign_coefs2matrix(matrix::Vector<matrix::Vector<matrix::Vector<Type, n_coeffs>, n_dofs>, n_int> &coeffs,\
size_t i_int, size_t i_dof, Type* input_1Darray, size_t n_coeffs_in)
{
	for (size_t i_coeff = 0; i_coeff < n_coeffs_in && i_coeff < n_coeffs; i_coeff++) coeffs(i_int)(i_dof)(i_coeff) = input_1Darray[i_coeff];
	return;
}

template <typename Type, size_t n_coeffs, size_t n_dofs, size_t n_int>
void assign_coefs2matrix(matrix::Vector<matrix::Vector<matrix::Vector<Type, n_coeffs>, n_dofs>, n_int> &coeffs,\
traj_file_data_t& data_in, size_t n_coeffs_in)
{
	for (size_t i_coeff = 0; i_coeff < n_coeffs_in && i_coeff < n_coeffs; i_coeff++) coeffs(data_in.i_int)(data_in.i_dof)(i_coeff) = data_in.coefs[i_coeff];
	return;
}

template <typename Type, size_t n_coeffs>
Type poly_val(matrix::Vector<Type, n_coeffs> &coeffs, Type time_int_s, Type tof_int_s, uint8_t deriv_order, size_t n_coeffs_in)
{
	Type out = static_cast<Type>(0.0);
	double tau = static_cast<double>(time_int_s) / static_cast<double>(tof_int_s);

	Type scaling = static_cast<Type>(pow(1.0 / static_cast<double>(tof_int_s), static_cast<double>(deriv_order)));
	if (n_coeffs_in > n_coeffs) n_coeffs_in = n_coeffs;

	if (deriv_order == 0) for (size_t i = 0; i < n_coeffs_in; i++) out += coeffs(i)*static_cast<Type>(pow(tau,static_cast<double>(i)));
	else
	{
		for (size_t i = static_cast<size_t>(deriv_order); i < n_coeffs_in; i++)
		{
			Type prod__ = static_cast<Type>(1.0);
			for (size_t ii = i - static_cast<size_t>(deriv_order) + 1; ii < i + 1; ii++) prod__ *= static_cast<Type>(ii);

			out += coeffs(i)*prod__*static_cast<Type>(pow(tau,static_cast<double>(i - static_cast<size_t>(deriv_order))));
		}
	}
	out *= scaling;
	return out;
}

template <typename Type, size_t n_coefs, size_t n_dofs, size_t n_int>
int eval_traj(matrix::Vector<Type, n_dofs> &eval_vec, Type time_trajectory_s, matrix::Vector<matrix::Vector<matrix::Vector<Type, n_coefs>, n_dofs>, n_int> &coeffs,\
 		matrix::Vector<Type, n_int> &tof_int_s, uint8_t deriv_order,\
		size_t n_coeffs_in, size_t n_dofs_in, size_t n_int_in)
{

	//locate segment number and tof for this segment:
	size_t i_int;
	if (n_int_in > n_int) n_int_in = n_int;
	Type TOF_max = static_cast<Type>(0.0);
	Type time_int_s = static_cast<Type>(0.0);
	for (size_t i = 0; i < n_int_in; i++) TOF_max += tof_int_s(i);

	int res = 0;

	if (time_trajectory_s < static_cast<Type>(0.0))
	{
		i_int = 0;
		//time_int_s = static_cast<Type>(0.0);
	}
	else if (time_trajectory_s >= TOF_max)
	{
		i_int = n_int_in - 1;
		time_int_s = tof_int_s(i_int);
		res = 1;
	}
	else
	{
		Type tof_sum = static_cast<Type>(0.0);
		i_int = 0;
		for (size_t i = 0; i < n_int_in; i++)
		{
			tof_sum += tof_int_s(i);
			if (time_trajectory_s > tof_sum) i_int++;
			else break;
		}

		if (i_int > n_int_in - 1)
		{
			i_int = n_int_in - 1;
			time_int_s = tof_int_s(i_int);
		}
		else if (i_int > 0)
		{
			time_int_s = time_trajectory_s;
			for (size_t i = 0; i < i_int; i++) time_int_s -= tof_int_s(i);
		}
		else time_int_s = time_trajectory_s;
	}

	if (n_dofs_in > n_dofs) n_dofs_in = n_dofs;

	for (size_t i_dof = 0; i_dof < n_dofs_in; i_dof++)
	{
		eval_vec(i_dof) = poly_val<Type,n_coefs>(coeffs(i_int)(i_dof), time_int_s, tof_int_s(i_int), deriv_order, n_coeffs_in);
	}

	return res;
}





template<typename Type>
void point<Type>::start(void)
{
	timestamp = hrt_absolute_time();
	return;
}
template<typename Type>
void point<Type>::reset(void)
{
	pos.setZero();
	vel.setZero();
	acc.setZero();
	jerk.setZero();
	snap.setZero();
	return;
}
template<typename Type>
double point<Type>::get_time_s(void)
{
	return get_dt_s_hrt(timestamp);
}
template<typename Type>
point<Type>::point(/* args */)
{
	reset();
}
template<typename Type>
point<Type>::~point()
{
}

int trajectory::set_home(void)
{
	//need to feed-in intial point:
	//_sim_inbound_sub.update(&sm_inbound); //assume this is already done

	initial_point.reset();
	initial_point.pos(0) = sm_inbound.data[XYZ_OFFSET_START_IND]; 	//x
	initial_point.pos(1) = sm_inbound.data[XYZ_OFFSET_START_IND+1]; //y
	initial_point.pos(2) = sm_inbound.data[XYZ_OFFSET_START_IND+2]; //z



	matrix::Quaternion<float> vehicle_attitude_quat(\
	sm_inbound.data[QUAT_OFFSET_START_IND],\
	sm_inbound.data[QUAT_OFFSET_START_IND+1],\
	sm_inbound.data[QUAT_OFFSET_START_IND+2],\
	sm_inbound.data[QUAT_OFFSET_START_IND+3]);

	matrix::Euler<float> vehicle_attitude_eul(vehicle_attitude_quat);
	initial_point.pos(3) = vehicle_attitude_eul.psi();		//yaw
	return 0;
}

int trajectory::reset_ref2state(void)
{

	//need to feed-in current point:
	_sim_inbound_sub.update(&sm_inbound);

	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> pos;
	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> vel;
	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> acc;

	pos.setZero();
	vel.setZero();
	acc.setZero();

	pos(0) = sm_inbound.data[XYZ_OFFSET_START_IND];   //x
	pos(1) = sm_inbound.data[XYZ_OFFSET_START_IND+1]; //y
	pos(2) = sm_inbound.data[XYZ_OFFSET_START_IND+2]; //z

	vel(0) = sm_inbound.data[XYZ_VEL_OFFSET_START_IND];   	//Vx
	vel(1) = sm_inbound.data[XYZ_VEL_OFFSET_START_IND+1]; 	//Vy
	vel(2) = sm_inbound.data[XYZ_VEL_OFFSET_START_IND+2]; 	//Vz
	vel(3) = sm_inbound.data[PQR_OFFSET_START_IND+2]; 	//yaw_rate
	acc(0) = sm_inbound.data[XYZ_ACC_OFFSET_START_IND];   	//ACCx
	acc(1) = sm_inbound.data[XYZ_ACC_OFFSET_START_IND+1]; 	//ACCy
	acc(2) = sm_inbound.data[XYZ_ACC_OFFSET_START_IND+2]; 	//ACCz


	matrix::Quaternion<float> vehicle_attitude_quat(\
	sm_inbound.data[QUAT_OFFSET_START_IND],\
	sm_inbound.data[QUAT_OFFSET_START_IND+1],\
	sm_inbound.data[QUAT_OFFSET_START_IND+2],\
	sm_inbound.data[QUAT_OFFSET_START_IND+3]);

	matrix::Euler<float> vehicle_attitude_eul(vehicle_attitude_quat);
	pos(3) = vehicle_attitude_eul.psi();			//yaw


	//publish new data:
	sim_guidance_trajectory_s smg_traj{};
	smg_traj.time_s = 0.0f;
	smg_traj.n_dofs = 0.0f;
	for (size_t i = 0; i < 3; i++)
	{
		smg_traj.position[i] = static_cast<float>(pos(i));
		smg_traj.velocity[i] = static_cast<float>(vel(i));
		smg_traj.acceleration[i] = static_cast<float>(acc(i));
		smg_traj.jerk[i] = 0.0f;
		smg_traj.snap[i] = 0.0f;
	}

	smg_traj.timestamp = hrt_absolute_time();
	_sim_guidance_trajecotry_pub.publish(smg_traj);

	//also publish it in array format:
	debug_array_s smg{};
	smg.timestamp = hrt_absolute_time();
	size_t tmp_ind = 0;
	smg.id = debug_array_s::SIMULINK_GUIDANCE_ID;
	char message_name[10] = "guidance";
	memcpy(smg.name, message_name, sizeof(message_name));
	smg.name[sizeof(smg.name) - 1] = '\0'; // enforce null termination

	smg.data[tmp_ind] = static_cast<float>(status.finished);
	tmp_ind++;
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(pos(i));
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(vel(i));
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(acc(i));
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = 0.0f;
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = 0.0f;
		tmp_ind++;
	}
	_sim_guidance_pub.publish(smg);
	return 0;
}

void trajectory::update(void)
{
	//fist, check the requests:
	sim_guidance_request_s smg_request{};
	if (_sim_guidance_request_sub.update(&smg_request))//new request has been published, need to process
	{
		if (smg_request.reset || smg_request.start || smg_request.stop || smg_request.start_execution) //ignore if all is false (no real requests)
		{
			if (smg_request.reset) reset(); //check reset flag first
			if (smg_request.set_home) //process manually-requested set_home (even if disabled)
			{
				reset_ref2state(); //make sure we have the most recent state first
				set_home(); //set home to current state
				//this will only publish once, if trajectory execution is not enabled
				//won't do much, but is usefull if trying to capture current state
			}
			if (smg_request.start) //this is when we have received the first request (assume pos_hold is not yet enabled, but will be as we send back ack)
			{
				start(); //start if not started yet
				reset_ref2state(); //make sure we have the most recent state first
				set_home(); //set home to current state
				//assume pos_hold will latch on this point, so we won't update the ref untill trajectory is executed or reset
			}
			if (smg_request.stop) status.finished = true; //this this as early termination

			if (!smg_request.start && !smg_request.reset && !smg_request.stop && smg_request.start_execution \
				&& status.loaded && status.started && !status.finished)
			{
				status.executing = true; //trajecotry is fully loaded and ready, so start evaluation
				PX4_INFO("Started trajectory execution");
				initial_point.start(); //starts the timer for trajectory execuition
			}

			sim_guidance_request_s smg_request_ack{};
			smg_request_ack.reset = false;
			smg_request_ack.start = false;
			smg_request_ack.stop = false;
			smg_request_ack.start_execution = false;
			smg_request_ack.set_home = false;
			smg_request_ack.timestamp = hrt_absolute_time();

			_sim_guidance_request_pub.publish(smg_request_ack);
		}

	}//otherwise keep doing stuff


	if (status.started && !status.finished)
	{

		if (!status.loaded)
		{
			status.trajectory_valid = false;
			if (load() < 0)
			{
				PX4_INFO("Failed to load trajectory, disengaging guidance...");
				status.finished = true;
			}
		}
		else
		{
			if (status.executing && execute() < 0)
			{
				PX4_INFO("Failed to execute trajectory, disengaging guidance...");
				status.finished = true;
				status.executing = false;
			}
			else status.trajectory_valid = true;
		}
	}
	else
	{
		status.executing = false;
		status.trajectory_valid = false;
		//don't update the reference since it has home + ref untill guidance if properly reset
	}
	//if (!status.started) reset_ref2state(); //keep updating such that ref=state, but only when guidance was not engaged


	//publish guidance status:
	sim_guidance_status_s smg_status{};
	smg_status.started = status.started;
	smg_status.loaded = status.loaded;
	smg_status.executing = status.executing;
	smg_status.finished = status.finished;
	smg_status.trajectory_valid = status.trajectory_valid;
	smg_status.timestamp = hrt_absolute_time();

	_sim_guidance_status_pub.publish(smg_status);

	return;
}

/*
int file_loader_backend::create_test_file(const char* location)
{
	PX4_INFO("Begin trajectory loading sequence...");
	//read first row to get the settings of the trajectory:
	n_coeffs = 10;
	n_int = 2;
	n_dofs = 1;

	//check if all good:
	if (n_coeffs > n_coeffs_max)
	{
		PX4_INFO("Too many coefficients");
		return -1;
	}
	if (n_int > n_int_max)
	{
		PX4_INFO("Too many segments");
		return -1;
	}
	if (n_dofs > n_dofs_max)
	{
		PX4_INFO("Too many dofs");
		return -1;
	}
	if (n_coeffs == 0 || n_int == 0 || n_dofs == 0)
	{
		PX4_INFO("Ivalid trajectory (zeros in the settings)");
		return -1;
	}

	//load the trajectory data (we need coefficients and time allocated for each interval)
	DATATYPE_TRAJ tof_int_raw[n_int_max] = {2.107510627081439, 2.892489372918561};
	DATATYPE_TRAJ x0_coefs_raw[n_coeffs_max] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	DATATYPE_TRAJ y0_coefs_raw[n_coeffs_max] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	DATATYPE_TRAJ z0_coefs_raw[n_coeffs_max] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	DATATYPE_TRAJ yaw0_coefs_raw[n_coeffs_max] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};

	DATATYPE_TRAJ x1_coefs_raw[n_coeffs_max] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	DATATYPE_TRAJ y1_coefs_raw[n_coeffs_max] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	DATATYPE_TRAJ z1_coefs_raw[n_coeffs_max] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	DATATYPE_TRAJ yaw1_coefs_raw[n_coeffs_max] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};

	tof_int = matrix::Vector<DATATYPE_TRAJ, n_int_max>(tof_int_raw);
	coefs.setZero();

	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 0, 0, x0_coefs_raw, n_coeffs);
	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 0, 1, y0_coefs_raw, n_coeffs);
	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 0, 2, z0_coefs_raw, n_coeffs);
	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 0, 3, yaw0_coefs_raw, n_coeffs);

	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 1, 0, x1_coefs_raw, n_coeffs);
	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 1, 1, y1_coefs_raw, n_coeffs);
	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 1, 2, z1_coefs_raw, n_coeffs);
	assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, 1, 3, yaw1_coefs_raw, n_coeffs);
	return 0;
}
*/

int trajectory::load(void)
{
	file_loader.close_file(); //do this no matter what
	PX4_INFO("Begin trajectory loading sequence...");
	//read first row to get the settings of the trajectory:
	traj_file_header_t traj_header{};
	if (file_loader.read_header(traj_header) < 0) return -1;
	n_coeffs = static_cast<size_t>(traj_header.n_coeffs);
	n_int = static_cast<size_t>(traj_header.n_int);
	n_dofs = static_cast<size_t>(traj_header.n_dofs);

	//check if all good:
	if (n_coeffs > n_coeffs_max)
	{
		PX4_INFO("Too many coefficients");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_int > n_int_max)
	{
		PX4_INFO("Too many segments");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_dofs > n_dofs_max)
	{
		PX4_INFO("Too many dofs");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_coeffs == 0 || n_int == 0 || n_dofs == 0)
	{
		PX4_INFO("Ivalid trajectory (zeros in the settings)");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}

	//load the trajectory data (we need coefficients and time allocated for each interval)
	tof_int.setZero();
	coefs.setZero();
	matrix::Vector<DATATYPE_TRAJ, n_dofs_max> tof_int_i;
	for (size_t i_int = 0; i_int < n_int; i_int++)
	{
		tof_int_i.setZero();
		for (size_t i_dof = 0; i_dof < n_dofs; i_dof++)
		{
			traj_file_data_t traj_data{};
			if (file_loader.read_data(traj_data) < 0) return -1;

			//perform additional checks:
			if (static_cast<size_t>(traj_data.i_dof) != i_dof)
			{
				PX4_ERR("Error in the trajectory loading: i_dof for i_int=%u, i_dof=%u does not match the file.",static_cast<uint16_t>(i_int), static_cast<uint16_t>(i_dof));
				status.loaded = false;
				file_loader.close_file();
				return -1;
			}
			if (static_cast<size_t>(traj_data.i_int) != i_int)
			{
				PX4_ERR("Error in the trajectory loading: i_int for i_int=%u, i_dof=%u does not match the file.", static_cast<uint16_t>(i_int), static_cast<uint16_t>(i_dof));
				status.loaded = false;
				file_loader.close_file();
				return -1;
			}
			//that's all we can do for the data (as of right now)
			assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, traj_data, n_coeffs);
			tof_int_i(i_dof) = traj_data.t_int;
		}
		if (n_dofs > 1)
		{
			for (size_t i_dof = 0; i_dof < n_dofs-1; i_dof++)
			{
				if (fabsf(static_cast<float>(tof_int_i(i_dof) - tof_int_i(i_dof+1))) > 1.0E-5f)
				{
					PX4_ERR("Error in the trajectory loading: i_int=%u, t_int does not match accross all dofs.", static_cast<uint16_t>(i_int));
					status.loaded = false;
					file_loader.close_file();
					return -1;
				}
			}
		}
		tof_int(i_int) = tof_int_i(0);


	}

	status.loaded = true;
	file_loader.close_file();
	PX4_INFO("Trajectory successfully loaded!");
	return 0;
}
//#define DEBUG

int trajectory::execute(void)
{


	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> pos;
	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> vel;
	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> acc;
	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> jerk;
	matrix::Vector<DATATYPE_TRAJ,n_dofs_max> snap;
	pos.setZero();
	vel.setZero();
	acc.setZero();
	jerk.setZero();
	snap.setZero();

	double time_trajecotry_s = initial_point.get_time_s();

	int res = eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(pos, time_trajecotry_s, coefs, tof_int, 0, n_coeffs, n_dofs, n_int);
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(vel, time_trajecotry_s, coefs, tof_int, 1, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(acc, time_trajecotry_s, coefs, tof_int, 2, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(jerk, time_trajecotry_s, coefs, tof_int, 3, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(snap, time_trajecotry_s, coefs, tof_int, 4, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (res < 0) return -1;
	else if (res == 1)
	{
		status.executing = false;
		status.finished = true;
		PX4_INFO("Completed trajectory execution");
	}

	#ifdef DEBUG
	printf("X=%9.6ff, X_vel=%9.6ff, X_acc=%9.6ff, X_jerk=%9.6ff, X_snap = %9.6ff\n", (double)pos(0), (double)vel(0), (double)acc(0), (double)jerk(0), (double)snap(0));
	#endif

	//publish new data:
	sim_guidance_trajectory_s smg_traj{};
	smg_traj.time_s = time_trajecotry_s;
	smg_traj.n_dofs = static_cast<uint8_t>(n_dofs);
	for (size_t i = 0; i < n_dofs; i++)
	{
		smg_traj.position[i] = static_cast<float>(pos(i)) + static_cast<float>(initial_point.pos(i));
		smg_traj.velocity[i] = static_cast<float>(vel(i)) + static_cast<float>(initial_point.acc(i));
		smg_traj.acceleration[i] = static_cast<float>(acc(i)) + static_cast<float>(initial_point.vel(i));
		smg_traj.jerk[i] = static_cast<float>(jerk(i)) + static_cast<float>(initial_point.jerk(i));
		smg_traj.snap[i] = static_cast<float>(snap(i)) + static_cast<float>(initial_point.snap(i));
	}

	//printf("initial_point.pos(0) = %f\n",(double)initial_point.pos(0));

	#ifdef DEBUG
	printf("X=%9.6ff, X_vel=%9.6ff, X_acc=%9.6ff, X_jerk=%9.6ff, X_snap = %9.6ff\n\n",\
	 (double)(smg_traj.position[0]),\
	 (double)smg_traj.velocity[0],\
	 (double)smg_traj.acceleration[0],\
	 (double)smg_traj.jerk[0],\
	 (double)smg_traj.snap[0]);
	 #endif

	smg_traj.timestamp = hrt_absolute_time();
	_sim_guidance_trajecotry_pub.publish(smg_traj);

	//also publish it in array format:
	debug_array_s smg{};
	smg.timestamp = hrt_absolute_time();
	size_t tmp_ind = 0;
	smg.id = debug_array_s::SIMULINK_GUIDANCE_ID;
	char message_name[10] = "guidance";
	memcpy(smg.name, message_name, sizeof(message_name));
	smg.name[sizeof(smg.name) - 1] = '\0'; // enforce null termination

	smg.data[tmp_ind] = static_cast<float>(status.finished);
	tmp_ind++;
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(pos(i)) + static_cast<float>(initial_point.pos(i));
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(vel(i)) + static_cast<float>(initial_point.vel(i));
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(acc(i)) + static_cast<float>(initial_point.acc(i));
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(jerk(i)) + static_cast<float>(initial_point.jerk(i));
		tmp_ind++;
	}
	for (size_t i = 0; i < n_dofs_max; i++)
	{
		smg.data[tmp_ind] = static_cast<float>(snap(i)) + static_cast<float>(initial_point.snap(i));
		tmp_ind++;
	}
	_sim_guidance_pub.publish(smg);


	return 0;
}


void trajectory::start(void)
{
	if (!status.started)
	{
		reset();
		PX4_INFO("Initializing trajectory...");
		status.started = true;
	}

	return;
}
void trajectory::reset(void)
{
	initial_point.reset();
	status.started = false;
	status.executing = false;
	status.finished = false;
	status.trajectory_valid = false;


	n_coeffs = 0;
	n_int = 0;
	n_dofs = 0;
	return;
}

int trajectory::set_src(const char* _file)
{
	return set_src(file_loader.get_dir(), _file);
}

int trajectory::set_src(const char* _dir, const char* _file)
{
	if (file_loader.set_src(_file, _dir))
	{
		PX4_INFO("Failed to set source location at %s for file %s.", _dir, _file);
		return -1;
	}
	else
	{
		//much more efficient to just load file here
		status.loaded = false;
		load();
	}
	return 0;
}

void trajectory::print_status(void)
{
	PX4_INFO("Latched on to %s trajectory file in %s", file_loader.get_file(), file_loader.get_dir());

	PX4_INFO("Guidance Internal Status Report:");
	if (status.started) 	PX4_INFO("%-20s%10s", "Started:", "true");
	else 			PX4_INFO("%-20s%10s", "Started:", "false");

	if (status.loaded) 	PX4_INFO("%-20s%10s", "Loaded:", "true");
	else 			PX4_INFO("%-20s%10s", "Loaded:", "false");

	if (status.executing) 	PX4_INFO("%-20s%10s", "Executing:", "true");
	else 			PX4_INFO("%-20s%10s", "Executing:", "false");

	if (status.trajectory_valid) \
				PX4_INFO("%-20s%10s", "Trajectory valid:", "true");
	else 			PX4_INFO("%-20s%10s", "Trajectory valid:", "false");

	if (status.finished) 	PX4_INFO("%-20s%10s", "Finished:", "true");
	else 			PX4_INFO("%-20s%10s", "Finished:", "false");


	PX4_INFO("Latest Trajectory Parameters:");
	PX4_INFO("%-45s %u / %u", "Number of coefficients for each segment:", static_cast<uint16_t>(n_coeffs), static_cast<uint16_t>(n_coeffs_max));
	PX4_INFO("%-45s %u / %u", "Number of segments:", static_cast<uint16_t>(n_int), static_cast<uint16_t>(n_int_max));
	PX4_INFO("%-45s %u / %u", "Number of active degrees of freedom:", static_cast<uint16_t>(n_dofs), static_cast<uint16_t>(n_dofs_max));
	return;
}

trajectory::trajectory(/* args */)
{
}

trajectory::~trajectory()
{

}
