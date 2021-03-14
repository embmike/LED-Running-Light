/*
 * Led.h
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#ifndef LED_H_
#define LED_H_

#include "main.h"
#include "Receiver.h"
#include <functional>

class Led final : public Receiver
{

private:
	GPIO_TypeDef *port;
	uint16_t pin;

	std::function<void(void)> Led_PWM_Timer_Start;
	std::function<void(uint8_t)> Led_PWM_Timer_Reinit;
	std::function<void(void)> Led_PWM_Timer_Stop;


public:
	Led() = delete;

	Led(GPIO_TypeDef *port, uint16_t pin);

	Led(std::function<void(void)> _Led_PWM_Timer_Start,
			std::function<void(uint8_t)> _Led_PWM_Timer_Reinit,
			std::function<void(void)> _Led_PWM_Timer_Stop);

	virtual ~Led();

	void Receive_Ctrl_Button_PushedOn() override;
	void Receive_Ctrl_Button_PushedOff() override;
	void Receive_Joystick_Axis_Value(uint8_t adc_udx) override;
};

#endif /* LED_H_ */
