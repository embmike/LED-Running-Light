/*
 * Joystick.cpp
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#include "Joystick.h"

Joystick::Joystick(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig, SoftwareBus *sw_bus)
:hadc{hadc},
 sConfig{sConfig},
 sw_bus{sw_bus}
{}

Joystick::~Joystick()
{}

void Joystick::Compute()
{
	if(time_counter == 0)
	{
		uint8_t adc_udx = 0;
		uint8_t adc_udy = 0;
		time_counter = 50;

		sConfig->Channel = ADC_CHANNEL_5;
		HAL_ADC_ConfigChannel(hadc, sConfig);

		HAL_ADC_Start(hadc);
		if( HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK )
		{
			adc_udx = HAL_ADC_GetValue(hadc);
			sw_bus->Send_Joystick_X_Value(adc_udx);

		}
		HAL_ADC_Stop(hadc);

		sConfig->Channel = ADC_CHANNEL_6;
		HAL_ADC_ConfigChannel(hadc, sConfig);

		HAL_ADC_Start(hadc);
		if( HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK )
		{
			adc_udy = HAL_ADC_GetValue(hadc);
			sw_bus->Send_Joystick_Y_Value(adc_udy);
		}
		HAL_ADC_Stop(hadc);
	}
	else
	{
		// Debouncing
		time_counter = time_counter > 0 ? time_counter - 1 : 0;
	}

}

