/*
 * LEDStrip.h
 *
 *  Created on: Sep 13, 2020
 *      Author: miken
 */

#ifndef LEDSTRIP_H_
#define LEDSTRIP_H_

#include <array>
#include "main.h"
#include "Receiver.h"

class LED_Strip final : public Receiver
{

private:
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *spi_ena_store_port;
	uint16_t spi_ena_store_pin;
	uint8_t programm_id;
	uint32_t time_counter;
	uint8_t led_strip_value;
	uint8_t shift_direction;

	std::array<uint32_t, 5> time_base_arr{50, 100, 200, 500, 1000};
	uint32_t time_base_iter = 0;

	std::array<uint8_t, 11> programm5{0, 1, 3, 7, 14, 28, 56, 112, 224, 192, 128};
	std::array<uint8_t, 22> programm7{0, 1, 3, 7, 14, 28, 56, 112, 224, 192, 128, 0, 192, 224, 112, 56, 28, 14, 7, 3, 1};
	std::array<uint8_t, 9> programm9{0, 129, 66, 36, 24, 24, 36, 66, 129};
	uint32_t programm_iter = 0;


public:
	LED_Strip() = delete;
	LED_Strip(SPI_HandleTypeDef *hspi, GPIO_TypeDef *spi_ena_store_port, uint16_t spi_ena_store_pin);
	virtual ~LED_Strip();

	void Receive_Ctrl_Button_PushedOn() override;
	void Receive_Ctrl_Button_PushedOff() override;

	void Receive_Time_Button_PushedOn() override;
	void Receive_Time_Button_PushedOff() override;

	void Compute();

private:
	void Ctrl_Button_Pushed();
	void Time_Button_Pushed();
	void Shift_LED_Strip(uint8_t pData[]);

};

#endif /* LEDSTRIP_H_ */
