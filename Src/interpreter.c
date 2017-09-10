/*
 * interpreter.c
 *
 *  Created on: Sep 9, 2017
 *      Author: Robin
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stepper.h"


float parse_number(const char *line, char code, int def)
{
	const char *ptr = line;

	while (ptr && *ptr && ptr < line + strlen(line)) {
		if (*ptr == code) {
			return atof(ptr + 1);
		}
		ptr = strchr(ptr, ' ') + 1;
	}

	return def;
}

int extract_var(const char *line, char code, float *result)
{
	const char *ptr = line;

	while (ptr && *ptr && ptr < (line + strlen(line))) {
		if (*ptr == code) {
			*result = atof(ptr + 1);
			return 1;
		}
		ptr = strchr(ptr, ' ') + 1;
	}

	return 0;
}

void process_line(const char *line)
{
	double arg_x = 0;
	double arg_y = 0;
	double arg_z = 0;
	double arg_f = 0;

	if (line[0] == 'G') {
		switch ((int)parse_number(line, 'G', -1)) {
		case 0:
			if (extract_var(line, 'X', &arg_x) == 0) {
				arg_x = stepper.machine_coord.x;
			}

			if (extract_var(line, 'Y', &arg_y) == 0) {
				arg_y = stepper.machine_coord.y;
			}

			if (extract_var(line, 'Z', &arg_z) == 0) {
				arg_z = stepper.machine_coord.z;
			}

			arg_x = parse_number(line, 'X', tool_state.current_x);
			arg_y = parse_number(line, 'Y', tool_state.current_y);
			arg_z = parse_number(line, 'Z', tool_state.current_z);
			arg_f = parse_number(line, 'F', tool_state.current_feedrate);

			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			stepper_set_feedrate(arg_f);
			stepper_move_interpolated(arg_x, arg_y, arg_z);
			server_write("OK\r\n");
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

			// do move
			break;
		case 17:
			tool_state.current_plane = PLANE_XY;
			server_write("OK\r\n");
			break;
		case 18:
			tool_state.current_plane = PLANE_ZX;
			server_write("OK\r\n");
			break;
		case 19:
			tool_state.current_plane = PLANE_YZ;
			server_write("OK\r\n");
			break;
		case 28:
			stepper_home_axes();
			server_write("OK\r\n");
			break;
		default:
			server_write("ERR\r\n");
			break;
		}
	}
	else if (line[0] == 'F') {
		double feedrate = parse_number(line, 'F', tool_state.current_feedrate);
	}
}
