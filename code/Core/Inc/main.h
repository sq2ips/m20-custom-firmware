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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum { false, true } bool;
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
#define LPS_CS_Pin GPIO_PIN_4
#define LPS_CS_GPIO_Port GPIOA
#define TEMP_ADC_Pin GPIO_PIN_4
#define TEMP_ADC_GPIO_Port GPIOC
#define Temp_EN_Pin GPIO_PIN_5
#define Temp_EN_GPIO_Port GPIOC
#define BAT_ADC_Pin GPIO_PIN_0
#define BAT_ADC_GPIO_Port GPIOB
#define RF_Boost_Pin GPIO_PIN_12
#define RF_Boost_GPIO_Port GPIOB
#define ADF_TX_Data_Pin GPIO_PIN_13
#define ADF_TX_Data_GPIO_Port GPIOB
#define GPS_ON_Pin GPIO_PIN_14
#define GPS_ON_GPIO_Port GPIOB
#define RADIO_EN_Pin GPIO_PIN_15
#define RADIO_EN_GPIO_Port GPIOB
#define ADF_CLK_Pin GPIO_PIN_7
#define ADF_CLK_GPIO_Port GPIOC
#define ADF_Data_Pin GPIO_PIN_8
#define ADF_Data_GPIO_Port GPIOC
#define ADF_LE_Pin GPIO_PIN_9
#define ADF_LE_GPIO_Port GPIOC
#define POWER_ON_Pin GPIO_PIN_12
#define POWER_ON_GPIO_Port GPIOA
#define ADF_CE_Pin GPIO_PIN_3
#define ADF_CE_GPIO_Port GPIOB
#define Temp_475k_Pin GPIO_PIN_4
#define Temp_475k_GPIO_Port GPIOB
#define Temp_36_5k_Pin GPIO_PIN_5
#define Temp_36_5k_GPIO_Port GPIOB
#define Temp_12_1k_Pin GPIO_PIN_6
#define Temp_12_1k_GPIO_Port GPIOB
#define Temp_2M_Pin GPIO_PIN_8
#define Temp_2M_GPIO_Port GPIOB
#define Temp_330k_Pin GPIO_PIN_9
#define Temp_330k_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
