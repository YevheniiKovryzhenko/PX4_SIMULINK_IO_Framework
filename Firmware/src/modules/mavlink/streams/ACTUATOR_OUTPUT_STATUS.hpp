/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef ACTUATOR_OUTPUT_STATUS_HPP
#define ACTUATOR_OUTPUT_STATUS_HPP

#include <uORB/topics/actuator_outputs.h>

template <int N>
class MavlinkStreamActuatorOutputStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamActuatorOutputStatus<N>(mavlink); }

	static constexpr const char *get_name_static()
	{
		switch (N)
		{
		case 1:
			return "ACTUATOR_OUTPUT_STATUS_SV";
		case 2:
			return "ACTUATOR_OUTPUT_STATUS_ESC";
		default:
			return "ACTUATOR_OUTPUT_STATUS";
		}

	}
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _act_output_sub->advertised() ? (MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:

	explicit MavlinkStreamActuatorOutputStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		// XXX this can be removed once the multiplatform system remaps topics
		switch (N) {
		case 1:
			_act_output_sub = new uORB::Subscription{ORB_ID(actuator_outputs_sv)};
			break;

		case 2:
			_act_output_sub = new uORB::Subscription{ORB_ID(actuator_outputs_esc)};
			break;

		default:
			_act_output_sub = new uORB::Subscription{ORB_ID(actuator_outputs)};
			break;
		}
	}

	~MavlinkStreamActuatorOutputStatus() override
	{
		delete _act_output_sub;
	}

	uORB::Subscription *_act_output_sub{nullptr};


	bool send() override
	{
		actuator_outputs_s act;

		if (_act_output_sub && _act_output_sub->update(&act)) {
			mavlink_actuator_output_status_t msg{};

			msg.time_usec = act.timestamp;
			msg.active = act.noutputs;

			static size_t actuator_outputs_size = act.noutputs;
			static constexpr size_t mavlink_actuator_output_status_size = sizeof(msg.actuator) / sizeof(msg.actuator[0]);

			for (unsigned i = 0; i < math::min(actuator_outputs_size, mavlink_actuator_output_status_size); i++) {
				msg.actuator[i] = act.output[i];
			}

			mavlink_msg_actuator_output_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // ACTUATOR_OUTPUT_STATUS_HPP
