/*
 * stepper.c
 *
 *  Created on: Sep 4, 2017
 *      Author: Robin
 */

#include "stepper.h"
#include "gpio.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "event_groups.h"

#include <stdbool.h>
#include <math.h>
#include <string.h>


EventGroupHandle_t xStepperEventGroup;
EventGroupHandle_t xLimitSwitchEventGroup;
struct StepperState stepper;

bool stepper_channel_enabled(unsigned int ch);
bool stepper_channel_active(unsigned int ch);
unsigned int speed_to_step_period(unsigned int ch, float speed);
inline void stepper_set_clk(unsigned int ch, unsigned int state);
inline void stepper_set_dir(unsigned int ch, unsigned int state);
inline void stepper_reset_channel(unsigned int ch);


void stepper_init(void)
{
	//
	// initialize event groups and queues
	//
	xStepperEventGroup = xEventGroupCreate();
	xLimitSwitchEventGroup = xEventGroupCreate();

	//
	// clear stepper state struct
	//
	memset((void *)&stepper, 0, sizeof(stepper));
}

void stepper_channel_set_enabled(unsigned int ch, bool enabled)
{
	stepper.ch[ch].enabled = enabled;
}

void stepper_channel_set_axis(unsigned int ch, enum Axis axis)
{
	stepper.ch[ch].axis = axis;
}

void stepper_channel_set_reverse(unsigned int ch, bool reverse)
{
	stepper.ch[ch].reverse = reverse;
}

void stepper_channel_set_steps_per_mm(unsigned int ch, unsigned int steps)
{
	stepper.ch[ch].steps_per_mm = steps;
}

void stepper_channel_set_clk_port(unsigned int ch, GPIO_TypeDef *port, uint8_t pin)
{
	stepper.ch[ch].port_clk = port;
	stepper.ch[ch].pin_clk = pin;
}

void stepper_channel_set_dir_port(unsigned int ch, GPIO_TypeDef *port, uint8_t pin)
{
	stepper.ch[ch].port_dir = port;
	stepper.ch[ch].pin_dir = pin;
}

void stepper_channel_set_en_port(unsigned int ch, GPIO_TypeDef *port, uint8_t pin)
{
	stepper.ch[ch].port_en = port;
	stepper.ch[ch].pin_en = pin;
}

bool stepper_channel_enabled(unsigned int ch)
{
	return stepper.ch[ch].enabled;
}

bool stepper_channel_active(unsigned int ch)
{
	return stepper.ch[ch].active;
}

void stepper_channel_set_active(unsigned int ch)
{
	stepper.ch[ch].active = true;
}

/**
 * Set the stepper resolution in steps per mm.
 * @param ch Stepper motor channel
 * @param steps Number of steps required to move 1 mm
 */
void stepper_chan_set_steps_per_mm(unsigned int ch, unsigned int steps)
{
	stepper.ch[ch].steps_per_mm = steps;
}

unsigned int stepper_chan_steps_per_mm(unsigned int ch)
{
	return stepper.ch[ch].steps_per_mm;
}

unsigned int distance_to_steps(unsigned int steps_per_mm, float distance)
{
	return (unsigned int)roundf(distance * steps_per_mm);
}

/**
 * Given a stepper channel, convert speed to step period
 * based on the timer frequency.
 * @param ch Stepper motor channel
 * @param speed Speed in mm/s
 * @return Step period needed to move stepper at desired speed
 */
unsigned int speed_to_step_period(unsigned int steps_per_mm, float speed)
{
	float steps_per_sec = speed * steps_per_mm;
	float steps_spacing = STEPPER_TIMER_FREQ / steps_per_sec;
	return (unsigned int)roundf(steps_spacing);
}

/**
 * Set the state of the CLK pin for a stepper channel
 * @param ch Stepper motor channel
 * @param state 1 for set, 0 for reset
 */
inline void stepper_set_clk(unsigned int ch, unsigned int state)
{
	HAL_GPIO_WritePin(
			stepper.ch[ch].port_clk,
			stepper.ch[ch].pin_clk,
			state
			);
}

inline void stepper_set_dir(unsigned int ch, unsigned int state)
{
	HAL_GPIO_WritePin(
			stepper.ch[ch].port_dir,
			stepper.ch[ch].pin_dir,
			state
			);
}

inline void stepper_reset_channel(unsigned int ch)
{
	stepper.ch[ch].active = 0;
	stepper.ch[ch].step_counter = 0;
	stepper.ch[ch].step_index = 0;
	stepper.ch[ch].step_period = 0;
	stepper.ch[ch].step_target = 0;
	stepper.ch[ch].period_counter = 0;
}

void stepper_channel_setup_linear(struct StepperChannel *ch, float current, float target, float dist, float speed)
{
	if (target > current) {
		ch->direction = ch->reverse ? STEPPER_DIR_CCW : STEPPER_DIR_CW;
	}
	else {
		ch->direction = ch->reverse ? STEPPER_DIR_CW : STEPPER_DIR_CCW;
	}

	ch->step_target = distance_to_steps(ch->steps_per_mm, dist);
	ch->step_period = speed_to_step_period(ch->steps_per_mm, speed);
	ch->active = true;
}

bool stepper_move_interpolated(float x, float y, float z)
{
	EventBits_t stepper_bits = 0;
	EventBits_t waiting_bits = 0;
	struct StepperChannel *ch;
	int i;

	float linear_dist = sqrt(
			pow((stepper.machine_coord.x - x), 2) +
			pow((stepper.machine_coord.y - y), 2) +
			pow((stepper.machine_coord.z - z), 2));

	float linear_time = linear_dist / stepper.feedrate_current;

	float x_distance = fabs(stepper.machine_coord.x - x);
	float y_distance = fabs(stepper.machine_coord.y - y);
	float z_distance = fabs(stepper.machine_coord.z - z);

	float x_speed = x_distance / linear_time;
	float y_speed = y_distance / linear_time;
	float z_speed = z_distance / linear_time;

	for (i = 0; i < STEPPER_NUM_CHANNELS; i++)
	{
		ch = (struct StepperChannel *)&stepper.ch[i];

		if (ch->enabled)
		{
			stepper_reset_channel(i);

			switch (ch->axis)
			{
				case XAxis:
				{
					if (x_distance > 0) {
						stepper_channel_setup_linear(ch, stepper.machine_coord.x, x, x_distance, x_speed);
						ch->active = true;
						stepper.machine_coord.x = x;
					}
					break;
				}
				case YAxis:
				{
					if (y_distance > 0) {
						stepper_channel_setup_linear(ch, stepper.machine_coord.y, y, y_distance, y_speed);
						ch->active = true;
						stepper.machine_coord.y = y;
					}
					break;
				}
				case ZAxis:
				{
					if (z_distance > 0) {
						stepper_channel_setup_linear(ch, stepper.machine_coord.z, z, z_distance, z_speed);
						ch->active = true;
						stepper.machine_coord.z = z;
					}
					break;
				}
			}

			stepper_bits |= (1 << i);
		}
	}

	if (stepper_bits != 0) {
		HAL_TIM_Base_Start_IT(&htim2);

		/*
		 * Wait for the activated steppers to complete their movements
		 */
		while (true) {
			waiting_bits = xEventGroupWaitBits(xStepperEventGroup, stepper_bits, pdTRUE, pdFALSE, portMAX_DELAY);

			if ((waiting_bits >> STEPPER_NUM_CHANNELS) != 0) {
				/*
				 * Limit switch triggered, failure!
				 */
				HAL_TIM_Base_Stop_IT(&htim2);
			}

			if (xEventGroupWaitBits(xStepperEventGroup, stepper_bits, pdTRUE, pdFALSE, portMAX_DELAY) == stepper_bits) {
				HAL_TIM_Base_Stop_IT(&htim3);
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case LIM_X1_Pin:
		break;
	case LIM_X2_Pin:
		break;
	case LIM_Y1_Pin:
		break;
	case LIM_Y2_Pin:
		break;
	}
}

void stepper_timer_callback(void)
{
	int i = 0;
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	for (; i < STEPPER_NUM_CHANNELS; i++) {
		struct StepperChannel *ch = &stepper.ch[i];

		if (stepper_channel_active(i)) {
			stepper_set_clk(i, GPIO_PIN_RESET);

			/*
			 * Stepper has reached the end of its desired
			 * travel and will be reset.
			 */
			if (ch->step_counter == ch->step_target) {
				stepper_reset_channel(i);

				/* notify thread */

				xResult = xEventGroupSetBitsFromISR(
						xStepperEventGroup,
						(1 << i),
						&xHigherPriorityTaskWoken
						);

				if (xResult != pdFAIL) {
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
			/*
			 * Period has elapsed, pulse the stepper.
			 */
			else if (ch->period_counter == ch->step_period) {
				ch->period_counter = 0;
				ch->step_counter++;
				stepper_set_dir(i, ch->direction);
				stepper_set_clk(i, GPIO_PIN_SET);
			}
			/*
			 * Waiting for the next step cycle.
			 */
			else {
				ch->period_counter++;
			}
		}
	}
}
