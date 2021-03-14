/*
 * LEDStrip.cpp
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#include "LEDStrip.h"
#include <limits>


LED_Strip::LED_Strip(SPI_HandleTypeDef *hspi, GPIO_TypeDef *spi_ena_store_port, uint16_t spi_ena_store_pin)
:hspi{hspi},
 spi_ena_store_port{spi_ena_store_port},
 spi_ena_store_pin{spi_ena_store_pin},
 programm_id{0},
 time_counter{0},
 led_strip_value{0},
 shift_direction{0}
{}

LED_Strip::~LED_Strip()
{}

void LED_Strip::Receive_Ctrl_Button_PushedOn()
{
	Ctrl_Button_Pushed();
}

void LED_Strip::Receive_Ctrl_Button_PushedOff()
{
	Ctrl_Button_Pushed();
}

void LED_Strip::Ctrl_Button_Pushed()
{
	programm_id = programm_id == 9 ? 0 : programm_id + 1;
}

void LED_Strip::Receive_Time_Button_PushedOn()
{
	Time_Button_Pushed();
}

void LED_Strip::Receive_Time_Button_PushedOff()
{
	Time_Button_Pushed();
}

void LED_Strip::Time_Button_Pushed()
{
	if( time_base_iter + 1 == time_base_arr.size() )
	{
		time_base_iter = 0;
	}
	else
	{
		time_base_iter++;
	}
}

void LED_Strip::Shift_LED_Strip(uint8_t pData[])
{
	HAL_GPIO_WritePin(spi_ena_store_port, spi_ena_store_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, pData, 1, 1000);
	HAL_GPIO_WritePin(spi_ena_store_port, spi_ena_store_pin, GPIO_PIN_SET);
}

void LED_Strip::Compute()
{

	// Led strip programs
	if(programm_id % 2 == 0 && time_counter == 0)
	{
		time_counter = time_base_arr[time_base_iter];
		led_strip_value = 0;
		programm_iter = 0;
		uint8_t pData[1] = {0};

		Shift_LED_Strip(pData);

	}
	else if(programm_id == 1 && time_counter == 0)
	{
		time_counter = time_base_arr[time_base_iter];
		uint8_t pData[1] = {led_strip_value};

		Shift_LED_Strip(pData);

		if(led_strip_value == std::numeric_limits<uint8_t>::max())
		{
			led_strip_value = 0;
		}
		else
		{
			led_strip_value++;
		}
	}
	else if(programm_id == 3 && time_counter == 0)
	{
		time_counter = time_base_arr[time_base_iter];
		uint8_t pData[1] = {led_strip_value};

		Shift_LED_Strip(pData);

		if(led_strip_value == std::numeric_limits<uint8_t>::max() && shift_direction == 0)
		{
			shift_direction = 1;
		}
		else if(led_strip_value < std::numeric_limits<uint8_t>::max() && shift_direction == 0 )
		{
			led_strip_value++;
		}
		else if(led_strip_value == std::numeric_limits<uint8_t>::min() && shift_direction == 1)
		{
			shift_direction = 0;
		}
		else if(led_strip_value > std::numeric_limits<uint8_t>::min() && shift_direction == 1 )
		{
			led_strip_value--;
		}
	}
	else if(programm_id == 5 && time_counter == 0)
	{
		time_counter = time_base_arr[time_base_iter];
		uint8_t pData[1] = {programm5[programm_iter]};

		Shift_LED_Strip(pData);

		if(programm_iter + 1 == programm5.size())
		{
			programm_iter = 0;
		}
		else
		{
			programm_iter++;
		}
	}
	else if(programm_id == 7 && time_counter == 0)
	{
		time_counter = time_base_arr[time_base_iter];
		uint8_t pData[1] = {programm7[programm_iter]};

		Shift_LED_Strip(pData);

		if(programm_iter + 1 == programm7.size())
		{
			programm_iter = 0;
		}
		else
		{
			programm_iter++;
		}
	}
	else if(programm_id == 9 && time_counter == 0)
	{
		time_counter = time_base_arr[time_base_iter];
		uint8_t pData[1] = {programm9[programm_iter]};

		Shift_LED_Strip(pData);

		if(programm_iter + 1 == programm9.size())
		{
			programm_iter = 0;
		}
		else
		{
			programm_iter++;
		}
	}

	time_counter = time_counter == 0 ? 0 : time_counter - 1;
}
