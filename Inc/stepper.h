/*
 * stepper.h
 *
 *  Created on: Sep 4, 2017
 *      Author: Robin
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include <stdbool.h>
#include "stm32f2xx_hal.h"
#include "stm32f2xx_hal_gpio.h"

#define STEPPER_NUM_CHANNELS	7
#define STEPPER_DIR_CW	0
#define STEPPER_DIR_CCW	1
#define STEPPER_TIMER_FREQ		20000
#define STEPPER_STEPS_PER_MM	40
#define STEPPER_MAX_FEED_RATE	50

enum Axis {
	XAxis,
	YAxis,
	ZAxis
};

struct StepperChannel {
	/* GPIO Ports and Pins */
	GPIO_TypeDef *port_clk;
	GPIO_TypeDef *port_dir;
	GPIO_TypeDef *port_en;
	uint8_t pin_clk;
	uint8_t pin_dir;
	uint8_t pin_en;

	/* Channel Configuration */
	enum Axis axis;
	unsigned int steps_per_mm;
	bool reverse;
	bool enabled;

	/* State variables */
	unsigned int period_counter;
	unsigned int step_counter;
	unsigned int step_target;
	unsigned int step_period;
	unsigned int step_index;
	unsigned int direction;
	bool active;
};

struct StepperCoord {
	long int x;
	long int y;
	long int z;
};

struct StepperState {
	struct StepperChannel ch[STEPPER_NUM_CHANNELS];
	struct StepperCoord machine_coord;
	struct StepperCoord tool_offset;
	struct StepperCoord work_offset;
	float feedrate_max;
	float feedrate_current;
};

/**
 * Initialize the stepper state machine.
 */
void stepper_init(void);

/**
 * Begin a stepping sequence. The function blocks until the
 * operation is complete.
 */
void stepper_begin(void);

/**
 * Stop a stepping sequence if in progress. The function immediately
 * returns.
 */
void stepper_stop(void);

/**
 * Set a channel enable flag
 * @param ch integer channel number (value between 0 to STEPPER_NUM_CHANNELS-1)
 * @param enabled
 */
void stepper_channel_set_enabled(unsigned int ch, bool enabled);

/**
 * Set a channel axis indicator.
 * @param ch integer channel number
 * @param axis which axis this channel is on
 */
void stepper_channel_set_axis(unsigned int ch, enum Axis axis);
void stepper_channel_set_reverse(unsigned int ch, bool reverse);
void stepper_channel_set_steps_per_mm(unsigned int ch, unsigned int steps);
void stepper_channel_set_clk_port(unsigned int ch, GPIO_TypeDef *port, uint8_t pin);
void stepper_channel_set_dir_port(unsigned int ch, GPIO_TypeDef *port, uint8_t pin);
void stepper_channel_set_en_port(unsigned int ch, GPIO_TypeDef *port, uint8_t pin);
void stepper_timer_callback(void);

#endif /* STEPPER_H_ */
