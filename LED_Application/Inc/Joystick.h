/*
 * Joystick.h
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include "main.h"
#include "SoftwareBus.h"

class Joystick
{

private:
	ADC_HandleTypeDef *hadc;
	ADC_ChannelConfTypeDef *sConfig;
	SoftwareBus *sw_bus;
	uint32_t time_counter = 50;

public:
	Joystick() = delete;
	Joystick(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig, SoftwareBus *sw_bus);
	virtual ~Joystick();

	void Compute();
};

#endif /* JOYSTICK_H_ */
