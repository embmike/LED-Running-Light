/*
 * SoftwareBus.h
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#ifndef SOFTWAREBUS_H_
#define SOFTWAREBUS_H_

#include <array>
#include "Receiver.h"

class SoftwareBus final
{

private:
	std::array<Receiver*, 2> ctrl_btn_receiver_arr{nullptr, nullptr};
	std::array<Receiver*, 1> time_btn_receiver_arr{nullptr};
	std::array<Receiver*, 1> joystick_x_receiver_arr{nullptr};
	std::array<Receiver*, 1> joystick_y_receiver_arr{nullptr};

public:
	SoftwareBus();
	virtual ~SoftwareBus();

	void Send_Ctrl_Button_PushedOn();
	void Send_Ctrl_Button_PushedOff();

	void Send_Time_Button_PushedOn();
	void Send_Time_Button_PushedOff();

	void Send_Joystick_X_Value(uint8_t adc_udx);
	void Send_Joystick_Y_Value(uint8_t adc_udy);

	void Add_Ctrl_Button_Receiver(uint32_t position, Receiver* receiver);
	void Add_Time_Button_Receiver(uint32_t position, Receiver* receiver);
	void Add_Joystick_X_Value_Receiver(uint32_t position, Receiver* receiver);
	void Add_Joystick_Y_Value_Receiver(uint32_t position, Receiver* receiver);

};

#endif /* SOFTWAREBUS_H_ */
