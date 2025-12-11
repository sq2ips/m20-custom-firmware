/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32l0xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_it.h"

#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#if HORUS_ENABLE
#include "fsk4.h"
#endif
#if APRS_ENABLE
#include "afsk.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVC_IRQn 0 */

	/* USER CODE END SVC_IRQn 0 */
	/* USER CODE BEGIN SVC_IRQn 1 */

	/* USER CODE END SVC_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */
	/* USER CODE END SysTick_IRQn 0 */

	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) {
	/* USER CODE BEGIN TIM2_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
		LL_TIM_ClearFlag_UPDATE(TIM2);
#if HORUS_ENABLE
		if (FSK4_Active) FSK4_timer_handler();
#endif
	}
	/* USER CODE END TIM2_IRQn 0 */
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt and DAC1/DAC2 underrun error interrupts.
 */
void TIM6_DAC_IRQHandler(void) {
	/* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
		LL_TIM_ClearFlag_UPDATE(TIM6);
#if LED_MODE == 2
		LED_Handler();
#endif
	}
	/* USER CODE END TIM6_DAC_IRQn 0 */
	/* USER CODE BEGIN TIM6_DAC_IRQn 1 */

	/* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
 * @brief This function handles TIM21 global interrupt.
 */
void TIM21_IRQHandler(void) {
	/* USER CODE BEGIN TIM21_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM21)) {
		LL_TIM_ClearFlag_UPDATE(TIM21);
#if APRS_ENABLE
		if (AFSK_Active) AFSK_timer_handler();
#endif
	}
	/* USER CODE END TIM21_IRQn 0 */
	/* USER CODE BEGIN TIM21_IRQn 1 */

	/* USER CODE END TIM21_IRQn 1 */
}

/**
 * @brief This function handles TIM22 global interrupt.
 */
void TIM22_IRQHandler(void) {
	/* USER CODE BEGIN TIM22_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM22)) {
		LL_TIM_ClearFlag_UPDATE(TIM22);
		main_loop();
	}
	/* USER CODE END TIM22_IRQn 0 */
	/* USER CODE BEGIN TIM22_IRQn 1 */

	/* USER CODE END TIM22_IRQn 1 */
}

/**
 * @brief This function handles LPUART1 global interrupt / LPUART1 wake-up interrupt through EXTI line 28.
 */
void LPUART1_IRQHandler(void) {
	/* USER CODE BEGIN LPUART1_IRQn 0 */
	GPS_Handler();
	/* USER CODE END LPUART1_IRQn 0 */
	/* USER CODE BEGIN LPUART1_IRQn 1 */

	/* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
