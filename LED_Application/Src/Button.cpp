/*
 * Button.cpp
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#include "Button.h"


Button::Button(GPIO_TypeDef *port, uint16_t pin,
		std::function<void(void)> btn_pushed_on,
		std::function<void(void)> btn_pushed_off)
: port{port},
  pin{pin},
  Button_PushedOn{btn_pushed_on},
  Button_PushedOFF{btn_pushed_off}
 {}

Button::~Button()
{}

void Button::Compute()
{
	uint8_t btn_state_temp = HAL_GPIO_ReadPin(port, pin);
	debounce_time = debounce_time > 0 ? debounce_time - 1 : 0;

	if(btn_state_temp && debounce_time == 0)
	{
		if(btn_state)
		{
			Button_PushedOFF();
			btn_state = 0;
		}
		else
		{
			Button_PushedOn();
			btn_state = 1;
		}

		debounce_state = 1;
		debounce_time = 400;

		// LED toggeln
	}
	else if(debounce_time == 0 && debounce_state == 1)
	{
		debounce_state = 2;

		// Wait for debouncing
	}
	else if(debounce_state == 2 && btn_state_temp == 0)
	{
		debounce_state = 0;

		// Button must be off (pull-down)
	}
}


