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
