/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_iwdg.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l0xx_it.h"
#include "fsk4.h"
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
void GPS_Handler(void);
void main_loop(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin LL_GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define LED_Pin LL_GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define LPS_CS_Pin LL_GPIO_PIN_4
#define LPS_CS_GPIO_Port GPIOA
#define BAT_ADC_Pin LL_GPIO_PIN_0
#define BAT_ADC_GPIO_Port GPIOB
#define RF_Boost_Pin LL_GPIO_PIN_12
#define RF_Boost_GPIO_Port GPIOB
#define ADF_TX_Data_Pin LL_GPIO_PIN_13
#define ADF_TX_Data_GPIO_Port GPIOB
#define GPS_ON_Pin LL_GPIO_PIN_14
#define GPS_ON_GPIO_Port GPIOB
#define RADIO_EN_Pin LL_GPIO_PIN_15
#define RADIO_EN_GPIO_Port GPIOB
#define ADF_CLK_Pin LL_GPIO_PIN_7
#define ADF_CLK_GPIO_Port GPIOC
#define ADF_Data_Pin LL_GPIO_PIN_8
#define ADF_Data_GPIO_Port GPIOC
#define ADF_LE_Pin LL_GPIO_PIN_9
#define ADF_LE_GPIO_Port GPIOC
#define POWER_ON_Pin LL_GPIO_PIN_12
#define POWER_ON_GPIO_Port GPIOA
#define ADF_CE_Pin LL_GPIO_PIN_3
#define ADF_CE_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
