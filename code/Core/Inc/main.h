/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define EXPANSION_1_Pin GPIO_PIN_0
#define EXPANSION_1_GPIO_Port GPIOC
#define EXPANSION_2_Pin GPIO_PIN_1
#define EXPANSION_2_GPIO_Port GPIOC
#define Heater_ADC_2_Pin GPIO_PIN_3
#define Heater_ADC_2_GPIO_Port GPIOC
#define IR_RX_Pin GPIO_PIN_1
#define IR_RX_GPIO_Port GPIOA
#define EXPANSION_4_Pin GPIO_PIN_2
#define EXPANSION_4_GPIO_Port GPIOA
#define EXPANSION_PS_Pin GPIO_PIN_3
#define EXPANSION_PS_GPIO_Port GPIOA
#define Temp_ADC_Pin GPIO_PIN_4
#define Temp_ADC_GPIO_Port GPIOC
#define Temp_EN_Pin GPIO_PIN_5
#define Temp_EN_GPIO_Port GPIOC
#define Heater_ADC_1_Pin GPIO_PIN_1
#define Heater_ADC_1_GPIO_Port GPIOB
#define RF_Boost_Pin GPIO_PIN_12
#define RF_Boost_GPIO_Port GPIOB
#define ADF_TX_Data_Pin GPIO_PIN_13
#define ADF_TX_Data_GPIO_Port GPIOB
#define GPS_ON_Pin GPIO_PIN_14
#define GPS_ON_GPIO_Port GPIOB
#define RADIO_EN_Pin GPIO_PIN_15
#define RADIO_EN_GPIO_Port GPIOB
#define TL555_in_Pin GPIO_PIN_6
#define TL555_in_GPIO_Port GPIOC
#define ADF_CLK_Pin GPIO_PIN_7
#define ADF_CLK_GPIO_Port GPIOC
#define ADF_Data_Pin GPIO_PIN_8
#define ADF_Data_GPIO_Port GPIOC
#define ADF_LE_Pin GPIO_PIN_9
#define ADF_LE_GPIO_Port GPIOC
#define DC_boost_Pin GPIO_PIN_12
#define DC_boost_GPIO_Port GPIOA
#define Heater_Pin GPIO_PIN_12
#define Heater_GPIO_Port GPIOC
#define Battery_on_Pin GPIO_PIN_2
#define Battery_on_GPIO_Port GPIOD
#define Trmp_R4_Pin GPIO_PIN_4
#define Trmp_R4_GPIO_Port GPIOB
#define Temp_R2_Pin GPIO_PIN_5
#define Temp_R2_GPIO_Port GPIOB
#define Temp_R1_Pin GPIO_PIN_6
#define Temp_R1_GPIO_Port GPIOB
#define EXPANSION_3_Pin GPIO_PIN_7
#define EXPANSION_3_GPIO_Port GPIOB
#define Temp_R5_Pin GPIO_PIN_8
#define Temp_R5_GPIO_Port GPIOB
#define Temp_R3_Pin GPIO_PIN_9
#define Temp_R3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
