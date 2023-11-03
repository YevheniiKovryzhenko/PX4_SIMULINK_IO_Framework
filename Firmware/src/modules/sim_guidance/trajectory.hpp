#pragma once

#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sim_guidance_trajectory.h>
#include <uORB/topics/sim_guidance_status.h>
#include <uORB/topics/sim_guidance_request.h>
#include <uORB/topics/debug_array.h>
#include "file_loader_backend.hpp"

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;


double get_dt_s_hrt(hrt_abstime &time_stamp);



template <typename Type>
class point
{
private:
	hrt_abstime	timestamp{0};
public:
	matrix::Vector<Type, 4> pos; // [x y z yaw]
	matrix::Vector<Type, 4> vel; // [x y z yaw]
	matrix::Vector<Type, 4> acc; // [x y z yaw]
	matrix::Vector<Type, 4> jerk; // [x y z yaw]
	matrix::Vector<Type, 4> snap; // [x y z yaw]

	void start(void);
	void reset(void);

	double get_time_s(void);

	point(/* args */);
	~point();
};

using pointf = point<float>;

class trajectory_type
{
public:
	size_t n_coefs;
	size_t n_dofs;
	size_t n_int;

	trajectory_type(size_t _n_coefs, size_t _n_dofs, size_t _n_int);
	~trajectory_type();
};




class trajectory
{
private:
	static const size_t n_coeffs_max = 10;
	static const size_t n_dofs_max = 4;
	static const size_t n_int_max = 50;

	size_t n_coeffs = 0;
	size_t n_int = 0;
	size_t n_dofs = 0;

	matrix::Vector<matrix::Vector<matrix::Vector<float, n_coeffs_max>, n_dofs_max>, n_int_max> coefs;
	matrix::Vector<float, n_int_max> tof_int;

	pointf initial_point{};


	void start(void);
	void reset(void);
	int load(void);
	int load_dummy_data(void);
	int execute(void);
	int set_home(void);
	int reset_ref2state(void);

	sim_guidance_status_s status{};
	debug_array_s sm_inbound{};


	// Publications
	uORB::Publication<sim_guidance_trajectory_s>	_sim_guidance_trajecotry_pub{ORB_ID(sim_guidance_trajectory)};
	uORB::Publication<sim_guidance_status_s>	_sim_guidance_status_pub{ORB_ID(sim_guidance_status)};
	uORB::Publication<sim_guidance_request_s>	_sim_guidance_request_pub{ORB_ID(sim_guidance_request)};
	uORB::Publication<debug_array_s>		_sim_guidance_pub{ORB_ID(simulink_guidance)};


	// Subscriptions
	uORB::Subscription				_sim_guidance_request_sub{ORB_ID(sim_guidance_request)};
	uORB::Subscription				_sim_inbound_sub{ORB_ID(simulink_inbound)};

public:
	trajectory(/* args */);
	~trajectory();

	file_loader_backend file_loader{};
	int set_src(const char* _file);
	int set_src(const char* _dir, const char* _file);

	void print_status(void);
	void update(void); //main update loop
};

