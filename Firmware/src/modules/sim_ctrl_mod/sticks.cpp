#include "sticks.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

char rc_map_stick(float &out, rc_channels_s& rc_ch, uint8_t ch)
{
	if (ch > rc_channels_s::FUNCTION_MAN) return -1;

	uint8_t ind = rc_ch.function[ch];
	if (static_cast<size_t>(ind) > sizeof(rc_ch.channels)) return -1;
	else {
		out = rc_ch.channels[ind];
		return 0;
	}
}


//char rc_channels2sticks_ind(uint8_t rc_ch, sticks_ind& sticks);

char sticks_ind2rc_channels(sticks_ind sticks, rc_channels_s& rc_ch, float& val)
{
	uint8_t rc_ch_ind = 0;
	switch (sticks)
	{
	case ROLL:
		rc_ch_ind = rc_channels_s::FUNCTION_ROLL;
		break;
	case PITCH:
		rc_ch_ind = rc_channels_s::FUNCTION_PITCH;
		break;
	case YAW:
		rc_ch_ind = rc_channels_s::FUNCTION_YAW;
		break;
	case THROTTLE:
		rc_ch_ind = rc_channels_s::FUNCTION_THROTTLE;
		break;
	case MODE:
		rc_ch_ind = rc_channels_s::FUNCTION_MODE;
		break;
	case ARMED:
		rc_ch_ind = rc_channels_s::FUNCTION_ARMSWITCH;
		break;
 	case AUX1:
		rc_ch_ind = rc_channels_s::FUNCTION_AUX_1;
		break;
	case AUX2:
		rc_ch_ind = rc_channels_s::FUNCTION_AUX_2;
		break;
	case AUX3:
		rc_ch_ind = rc_channels_s::FUNCTION_AUX_3;
		break;
	case AUX4:
		rc_ch_ind = rc_channels_s::FUNCTION_AUX_4;
		break;
	case AUX5:
		rc_ch_ind = rc_channels_s::FUNCTION_AUX_5;
		break;
	case AUX6:
		rc_ch_ind = rc_channels_s::FUNCTION_AUX_6;
		break;
	case EXTR1:
		val = get_extr(1);
		return 0;
	case EXTR2:
		val = get_extr(2);
		return 0;
	case EXTR3:
		val = get_extr(3);
		return 0;
	case EXTR4:
		val = get_extr(4);
		return 0;
	case EXTR5:
		val = get_extr(5);
		return 0;
	case EXTR6:
		val = get_extr(6);
		return 0;

	default:
		return -1;
	}

	return rc_map_stick(val, rc_ch, rc_ch_ind);
}

char sticks_ind2manual_control(sticks_ind sticks, manual_control_setpoint_s& man_control,float& val)
{
	switch (sticks)
	{
	case ROLL:
		val = man_control.y;
		break;
	case PITCH:
		val = man_control.x;
		break;
	case YAW:
		val = man_control.r;
		break;
	case THROTTLE:
		val = man_control.z;
		break;
	case AUX1:
		val = man_control.aux1;
		break;
	case AUX2:
		val = man_control.aux2;
		break;
	case AUX3:
		val = man_control.aux3;
		break;
	case AUX4:
		val = man_control.aux4;
		break;
	case AUX5:
		val = man_control.aux5;
		break;
	case AUX6:
		val = man_control.aux6;
		break;
	case EXTR1:
		val = get_extr(1);
		break;
	case EXTR2:
		val = get_extr(2);
		break;
	case EXTR3:
		val = get_extr(3);
		break;
	case EXTR4:
		val = get_extr(4);
		break;
	case EXTR5:
		val = get_extr(5);
		break;
	case EXTR6:
		val = get_extr(6);
		break;
	default:
		return -1;
	}
	return 0;
}


uint8_t get_input_ind(sticks_ind stick)
{

	char str[17];
	const char *prefix;

	prefix = "SM_IN_MAP_";
	sprintf(str, "%s%u", prefix, stick + 1);
	int32_t ind = 0;
	param_get(param_find(str), &ind);

	return static_cast<uint8_t>(ind);
}

uint8_t get_output_ind(uint8_t out_ind)
{

	char str[17];
	const char *prefix;

	prefix = "SM_OUT_MAP_";
	sprintf(str, "%s%u", prefix, out_ind + 1);
	int32_t ind = 0;
	param_get(param_find(str), &ind);

	return static_cast<uint8_t>(ind);
}

float get_extr(uint8_t i)
{
	char str[17];
	const char *prefix;

	prefix = "SM_EXTR";
	sprintf(str, "%s%u", prefix, i);
	float val = 0;
	param_get(param_find(str), &val);

	return val;
}
