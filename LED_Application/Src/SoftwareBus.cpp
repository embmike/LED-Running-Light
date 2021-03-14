/*
 * SoftwareBus.cpp
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#include "SoftwareBus.h"

SoftwareBus::SoftwareBus()
{}

SoftwareBus::~SoftwareBus()
{}

void SoftwareBus::Add_Ctrl_Button_Receiver(uint32_t position, Receiver* receiver)
{
	if( position < ctrl_btn_receiver_arr.size() )
	{
		ctrl_btn_receiver_arr[position] = receiver;
	}
}

void SoftwareBus::Add_Time_Button_Receiver(uint32_t position, Receiver* receiver)
{
	if( position < time_btn_receiver_arr.size() )
	{
		time_btn_receiver_arr[position] = receiver;
	}
}

void SoftwareBus::Add_Joystick_X_Value_Receiver(uint32_t position, Receiver* receiver)
{
	if( position < joystick_x_receiver_arr.size() )
	{
		joystick_x_receiver_arr[position] = receiver;
	}
}

void SoftwareBus::Add_Joystick_Y_Value_Receiver(uint32_t position, Receiver* receiver)
{
	if( position < joystick_y_receiver_arr.size() )
	{
		joystick_y_receiver_arr[position] = receiver;
	}
}

void SoftwareBus::Send_Ctrl_Button_PushedOn()
{
	for(auto iter : ctrl_btn_receiver_arr)
	{
		if( iter != nullptr)
		{
			iter->Receive_Ctrl_Button_PushedOn();
		}
	}
}

void SoftwareBus::Send_Ctrl_Button_PushedOff()
{
	for(auto iter : ctrl_btn_receiver_arr)
	{
		if( iter != nullptr)
		{
			iter->Receive_Ctrl_Button_PushedOff();
		}
	}
}

void SoftwareBus::Send_Time_Button_PushedOn()
{
	for(auto iter : time_btn_receiver_arr)
	{
		if( iter != nullptr)
		{
			iter->Receive_Time_Button_PushedOn();
		}
	}
}

void SoftwareBus::Send_Time_Button_PushedOff()
{
	for(auto iter : time_btn_receiver_arr)
	{
		if( iter != nullptr)
		{
			iter->Receive_Time_Button_PushedOn();
		}
	}
}

void SoftwareBus::Send_Joystick_X_Value(uint8_t adc_udx)
{
	for(auto iter : joystick_x_receiver_arr)
	{
		if( iter != nullptr)
		{
			iter->Receive_Joystick_Axis_Value(adc_udx);
		}
	}
}

void SoftwareBus::Send_Joystick_Y_Value(uint8_t adc_udy)
{
	for(auto iter : joystick_y_receiver_arr)
	{
		if( iter != nullptr)
		{
			iter->Receive_Joystick_Axis_Value(adc_udy);
		}
	}
}

