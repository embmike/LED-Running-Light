/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MX_PWM_TIM16_Start(void);
void MX_PWM_TIM16_Stop(void);
void MX_PWM_TIM16_Reinit(uint8_t adc_ud);
void MX_PWM_TIM17_Start(void);
void MX_PWM_TIM17_Stop(void);
void MX_PWM_TIM17_Reinit(uint8_t adc_ud);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define URY_Pin GPIO_PIN_0
#define URY_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define URX_Pin GPIO_PIN_4
#define URX_GPIO_Port GPIOA
#define UWX_PWM_Pin GPIO_PIN_6
#define UWX_PWM_GPIO_Port GPIOA
#define UWY_PWM_Pin GPIO_PIN_7
#define UWY_PWM_GPIO_Port GPIOA
#define LED_GREEN_1_CTRL_Pin GPIO_PIN_5
#define LED_GREEN_1_CTRL_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define BUTTON_2_NCC1_TIME_Pin GPIO_PIN_6
#define BUTTON_2_NCC1_TIME_GPIO_Port GPIOC
#define BUTTON_1_NCC1_CTRL_Pin GPIO_PIN_8
#define BUTTON_1_NCC1_CTRL_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SPI3_ENA_STORE_Pin GPIO_PIN_15
#define SPI3_ENA_STORE_GPIO_Port GPIOA
#define SPI3_SCK_SHIFT_Pin GPIO_PIN_10
#define SPI3_SCK_SHIFT_GPIO_Port GPIOC
#define SPI3_MOSI_DATA_Pin GPIO_PIN_12
#define SPI3_MOSI_DATA_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
