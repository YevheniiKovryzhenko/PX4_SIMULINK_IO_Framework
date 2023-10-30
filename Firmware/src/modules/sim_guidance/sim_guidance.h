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
#include "trajectory.hpp"

extern "C" __EXPORT int sim_guidance_main(int argc, char *argv[]);


class SIM_GUIDANCE : public ModuleBase<SIM_GUIDANCE>, public ModuleParams
{
public:

	SIM_GUIDANCE(int example_param);

	virtual ~SIM_GUIDANCE() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SIM_GUIDANCE *instantiate(int argc, char *argv[]);

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

	void update_guidance(void);

	trajectory traj{};

	hrt_abstime	_boot_timestamp{0};


	// Subscriptions
	uORB::Subscription		_parameter_update_sub{ORB_ID(parameter_update)};

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
		(ParamInt<px4::params::SMG_EN>) _param_smg_en
	)//MAKE SURE EVERY PARAMETER IS FOLLOWED BY "," AND LAST ONE DOES NOT HAVE ANYTHING
};

