/*
 * Receiver.h
 *
 *  Created on: 13.09.2020
 *      Author: miken
 */

#ifndef RECEIVER_H_
#define RECEIVER_H_

#include "main.h"

class Receiver
{

public:
	Receiver() {};
	virtual ~Receiver() {};

	virtual void Receive_Ctrl_Button_PushedOn() {};
	virtual void Receive_Ctrl_Button_PushedOff() {};
	virtual void Receive_Time_Button_PushedOn() {};
	virtual void Receive_Time_Button_PushedOff() {};

	virtual void Receive_Joystick_Axis_Value(uint8_t adc_ud) {};

};

#endif /* RECEIVER_H_ */
