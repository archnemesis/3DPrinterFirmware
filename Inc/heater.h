/*
 * heater.h
 *
 *  Created on: Sep 5, 2017
 *      Author: Robin
 */

#ifndef HEATER_H_
#define HEATER_H_

#include <stdbool.h>
#include "gpio.h"

#define HEATER_NUM_CHANNELS 2
#define HEATER_DELTA_SAMPLES 4

struct HeaterChannel {
	GPIO_TypeDef *port;
	unsigned int pin;
	float setpoint;
	float senseval;
	bool enabled;
	bool state;
	float sh_a;
	float sh_b;
	float sh_c;
	float p;
	float i;
	float d;
	float err;
	float err_accum;
	float delta_sum;
	int delta_idx;
	int delta_samples[HEATER_DELTA_SAMPLES];
};

struct HeaterState {
	struct HeaterChannel ch[2];
	unsigned int pending_channel;
	uint16_t adc_buffer[4];
};

void heater_init(void);
void heater_start(void);
void heater_stop(void);
void heater_start_conversion(void);
void heater_channel_set_coefficients(unsigned int ch, float a, float b, float c);
void heater_channel_set_temp(unsigned int ch, float temp);
void heater_channel_set_enabled(unsigned int ch, bool enabled);
void heater_channel_set_port(unsigned int ch, GPIO_TypeDef *port, unsigned int pin);

#endif /* HEATER_H_ */
