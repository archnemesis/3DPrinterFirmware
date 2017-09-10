/*
 * heater.c
 *
 *  Created on: Sep 6, 2017
 *      Author: Robin
 */

#include "heater.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include <string.h>
#include <math.h>

struct HeaterState heater;

void heater_init(void)
{
	memset(&heater, 0, sizeof(struct HeaterState));
}

void heater_start(void)
{
	HAL_TIM_Base_Start_IT(&htim5);
}

void heater_stop(void)
{
	HAL_TIM_Base_Stop(&htim5);
}

void heater_channel_set_port(unsigned int ch, GPIO_TypeDef *port, unsigned int pin)
{
	heater.ch[ch].port = port;
	heater.ch[ch].pin = pin;
}

void heater_channel_set_coefficients(unsigned int ch, float a, float b, float c)
{
	heater.ch[ch].sh_a = a;
	heater.ch[ch].sh_b = b;
	heater.ch[ch].sh_c = c;
}

float heater_channel_get_temp(unsigned int ch)
{
	return heater.ch[ch].senseval;
}

void heater_channel_set_temp(unsigned int ch, float temp)
{
	heater.ch[ch].setpoint = temp;
}

void heater_channel_set_enabled(unsigned int ch, bool enabled)
{
	heater.ch[ch].enabled = enabled;
}

void heater_start_conversion(void)
{
	HAL_StatusTypeDef result = HAL_ADC_Start_DMA(
			&hadc1,
			(uint32_t *)&heater.adc_buffer[0],
			HEATER_NUM_CHANNELS
			);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	unsigned int i;
	unsigned int adc;
	float r;
	float t;
	float e;
	float pterm;
	float iterm;
	float dterm;
	int delta;

	for (i = 0; i < HEATER_NUM_CHANNELS; i++) {
		struct HeaterChannel *ch = &heater.ch[i];

		if (ch->enabled) {
			adc = heater.adc_buffer[i];
			r = ((pow(2, 12) / adc) - 1) * 100000.0;
			t = 1.0 / (ch->sh_a + (ch->sh_b * log(r)) + (ch->sh_c * pow(log(r), 3)));
			t = t - 273.15;
			ch->senseval = t;

			e = ch->setpoint - ch->senseval;
			ch->err_accum += e;
			pterm = ch->p * e;
			iterm = ch->err_accum * i;
			delta = e - ch->err;
			ch->delta_sum -= ch->delta_samples[ch->delta_idx];
			ch->delta_samples[ch->delta_idx] = delta;
			ch->delta_sum += (float)delta;
			ch->delta_idx = (ch->delta_idx + 1) % HEATER_DELTA_SAMPLES;
			ch->err = e;

			if (ch->state == false && (ch->senseval < (ch->setpoint - 5.0))) {
				HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_SET);
				ch->state = true;
			}
			else if (ch->state == true && (ch->senseval >= ch->setpoint)) {
				HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_RESET);
				ch->state = false;
			}
		}
	}
}
