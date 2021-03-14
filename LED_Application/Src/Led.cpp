/*
 * Led.cpp
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#include "Led.h"

Led::Led(GPIO_TypeDef *port, uint16_t pin)
: port{port},
 pin{pin},
 Led_PWM_Timer_Start{nullptr},
 Led_PWM_Timer_Reinit{nullptr},
 Led_PWM_Timer_Stop{nullptr}
{}

Led::Led(std::function<void(void)> _Led_PWM_Timer_Start,
	     std::function<void(uint8_t)> _Led_PWM_Timer_Reinit,
	     std::function<void(void)> _Led_PWM_Timer_Stop)
: port{nullptr},
  pin{65535},
  Led_PWM_Timer_Start{_Led_PWM_Timer_Start},
  Led_PWM_Timer_Reinit{_Led_PWM_Timer_Reinit},
  Led_PWM_Timer_Stop{_Led_PWM_Timer_Stop}
{}

Led::~Led()
{}

void Led::Receive_Ctrl_Button_PushedOn()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void Led::Receive_Ctrl_Button_PushedOff()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void Led::Receive_Joystick_Axis_Value(uint8_t adc_ud)
{
	Led_PWM_Timer_Stop();
	Led_PWM_Timer_Reinit(adc_ud);
	Led_PWM_Timer_Start();
}

