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
	float arg_cmd = 0;
	float arg_x = 0;
	float arg_y = 0;
	float arg_z = 0;
	float arg_f = 0;
	char return_buffer[32];

	if (line[0] == 'G') {
		if (extract_var(line, 'G', &arg_cmd) == 0) {
			server_write("ERR\r\n");
			return;
		}

		switch ((unsigned int)arg_cmd) {
		case 0:
			if (extract_var(line, 'X', &arg_x) == 0) {
				arg_x = stepper_get_machine_x();
			}

			if (extract_var(line, 'Y', &arg_y) == 0) {
				arg_y = stepper_get_machine_y();
			}

			if (extract_var(line, 'Z', &arg_z) == 0) {
				arg_z = stepper_get_machine_z();
			}

			if (extract_var(line, 'F', &arg_f) == 0) {
				arg_f = stepper_get_current_feedrate();
			}

			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			stepper_set_feedrate(arg_f);
			stepper_move_interpolated(arg_x, arg_y, arg_z);
			server_write("OK\r\n");
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

			// do move
			break;
		case 17:
			stepper_set_active_plane(XYPlane);
			server_write("OK\r\n");
			break;
		case 18:
			stepper_set_active_plane(XZPlane);
			server_write("OK\r\n");
			break;
		case 19:
			stepper_set_active_plane(YZPlane);
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
	else if (line[0] == 'C') {
		if (extract_var(line, 'C', &arg_cmd) == 0) {
			server_write("ERR\r\n");
			return;
		}
		switch ((unsigned int)arg_cmd) {
		case 0:
			/* TODO: C0 reset configuration */
			break;
		case 1:
			/**
			 * C1: Set stepper channel configuration
			 * Example:
			 * 	C1 X0 S40 R1 A0 E1
			 * 	(X) Channel = 0
			 * 	(S) Steps per MM = 40 steps per mm
			 * 	(R) Reverse = 1 (Fwd=0, Rev=1)
			 * 	(A) Axis = X (X=0, Y=1, Z=2)
			 */

			float t;
			unsigned int ch;
			unsigned int u;

			if (extract_var(line, 'X', &t) == 0) {
				server_write("ERR\r\n");
				break;
			}

			ch = (unsigned int)t;

			if (ch < 0 || ch >= STEPPER_NUM_CHANNELS) {
				server_write("ERR\r\n");
			}

			if (extract_var(line, 'S', &t) == 1) {
				stepper_channel_set_steps_per_mm(ch, (unsigned int)t);
			}

			if (extract_var(line, 'R', &t) == 1) {
				if ((unsigned int)t > 1) {
					server_write("ERR\r\n");
					break;
				}
				stepper_channel_set_reverse(ch, (bool)t);
			}

			if (extract_var(line, 'A', &t) == 1) {
				if ((unsigned int)t > 2) {
					server_write("ERR\r\n");
					break;
				}
				stepper_channel_set_axis(ch, (enum Axis)t);
			}

			sprintf(return_buffer, "C1 X%d S%d D%d A%d\r\n",
					ch,
					stepper_channel_get_steps_per_mm(ch),
					(unsigned int)stepper_channel_get_reverse(ch),
					(unsigned int)stepper_channel_get_axis(ch));

			server_write(return_buffer);

			break;
		case 2:
			/**
			 * C2: Set limit switch configuration
			 * Example:
			 * 	C2 X0 A0 D1
			 * 	(X) Channel = 0
			 * 	(A) Axis = 0 (X=0, Y=1, Z=2)
			 * 	(D) Direction = 1 (Min=0, Max=1)
			 */
			break;
		}
	}
	else if (line[0] == 'F') {
		if (extract_var(line, 'F', &arg_f) == 1) {
			stepper_set_current_feedrate(arg_f);
			server_write("OK\r\n");
		}
	}
}
