/*
 * Button.h
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include "main.h"
#include "SoftwareBus.h"
#include <functional>

class Button final
{

private:
	GPIO_TypeDef *port;
	uint16_t pin;
	uint32_t debounce_time;
	uint8_t debounce_state;
	uint8_t btn_state;
	std::function<void(void)> Button_PushedOn;
	std::function<void(void)> Button_PushedOFF;



public:
	Button() = delete;
	Button(GPIO_TypeDef *port,
			uint16_t pin,
			std::function<void(void)> btn_pushed_on,
			std::function<void(void)> btn_pushed_off);
	virtual ~Button();

	void Compute();

};

#endif /* BUTTON_H_ */
