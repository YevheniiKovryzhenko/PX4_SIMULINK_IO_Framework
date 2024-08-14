/****************************************************************************
 *
 *    Copyright (C) 2024  Yevhenii Kovryzhenko. All rights reserved.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Affero General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Affero General Public License Version 3 for more details.
 *
 *    You should have received a copy of the
 *    GNU Affero General Public License Version 3
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions, and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions, and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *    3. No ownership or credit shall be claimed by anyone not mentioned in
 *       the above copyright statement.
 *    4. Any redistribution or public use of this software, in whole or in part,
 *       whether standalone or as part of a different project, must remain
 *       under the terms of the GNU Affero General Public License Version 3,
 *       and all distributions in binary form must be accompanied by a copy of
 *       the source code, as stated in the GNU Affero General Public License.
 *
 ****************************************************************************/

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

class UavcanServoController
{
public:
	static constexpr int MAX_ACTUATORS = actuator_outputs_s::NUM_ACTUATOR_OUTPUTS;
	static constexpr unsigned MAX_RATE_HZ = 50;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 6;	///< 0..31, inclusive, 0 - highest, 31 - lowest

	UavcanServoController(uavcan::INode &node);
	~UavcanServoController() = default;


	/*
	 * setup periodic updater
	 */
	int init(void);

	void update_params(void);

private:

	/*
	 * Setup timer and call back function for periodic updates
	 */
	void update_outputs(bool armed, bool fail, float outputs[MAX_ACTUATORS]);
	void update(const uavcan::TimerEvent &);
	typedef uavcan::MethodBinder<UavcanServoController *, void (UavcanServoController::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	uavcan::INode								&_node;
	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> _uavcan_pub_array_cmd;

	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	uORB::Subscription _actuator_outputs_sv_sub{ORB_ID(actuator_outputs_sv)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	bool sv_en_fl[MAX_ACTUATORS];
	bool sv_rev_fl[MAX_ACTUATORS];
	bool sv_esc_fl[MAX_ACTUATORS];
	int32_t sv_id[MAX_ACTUATORS];
	int32_t sv_min[MAX_ACTUATORS];
	int32_t sv_max[MAX_ACTUATORS];
	int32_t sv_trim[MAX_ACTUATORS];
	int32_t sv_disarm[MAX_ACTUATORS];
	int32_t sv_fail[MAX_ACTUATORS];

	bool disable_safety_checks_fl = false;
};
