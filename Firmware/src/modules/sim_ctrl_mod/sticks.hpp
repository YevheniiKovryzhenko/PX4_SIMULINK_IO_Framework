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

#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>

enum sticks_ind
{
	ROLL = 0,
	PITCH = 1,
	YAW = 2,
	THROTTLE = 3,
	ARMED = 4,
	MODE = 5,
	AUX1 = 6,
	AUX2 = 7,
	AUX3 = 8,
	AUX4 = 9,
	AUX5 = 10,
	AUX6 = 11, //reserved for wing angle control
	EXTR1 = 12,
	EXTR2 = 13,
	EXTR3 = 14,
	EXTR4 = 15,
	EXTR5 = 16,
	EXTR6 = 17
};


char sticks_ind2rc_channels(sticks_ind sticks, rc_channels_s& rc_ch, float& val);
char sticks_ind2manual_control(sticks_ind sticks, manual_control_setpoint_s& man_control,float& val);
uint8_t get_input_ind(sticks_ind sticks);
uint8_t get_output_ind(uint8_t out_ind);
float get_extr(uint8_t i);

char rc_map_stick(float &out, rc_channels_s& rc_ch, uint8_t ch);
