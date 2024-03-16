/*
 * adf7012.c
 *
 *  Created on: Mar 15, 2024
 *      Author: SQ2DK
 */
#include "adf7012.h"
#include "main.h"



void ADF_WriteReg(uint32_t val)
{
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, 0);
	HAL_Delay(0);

	HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, 0);
	HAL_Delay(0);

	for(uint8_t i=0; i<32; i++)
	{
		HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, (val>>(31-i))&1);
		for(uint16_t i=0; i<100; i++)
			asm("NOP");
		HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, 1);
		for(uint16_t i=0; i<100; i++)
			asm("NOP");
		HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, 0);
		for(uint16_t i=0; i<100; i++)
			asm("NOP");
	}

	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, 1);
	//HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, 1);
	for(uint16_t i=0; i<100; i++)
		asm("NOP");
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, 0);
	//HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, 0);
	for(uint16_t i=0; i<100; i++)
		asm("NOP");
}

void ADF_Reset(void)
{
	//example ADF7021 init
	HAL_GPIO_WritePin(RADIO_EN_GPIO_Port, RADIO_EN_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RADIO_EN_GPIO_Port, RADIO_EN_Pin, 1);

	HAL_Delay(100);
}

void myspi(uint32_t data)
{
	//uint32_t delay = 0;
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_RESET);
	//HAL_Delay(delay);
	__HAL_RCC_GPIOA_CLK_ENABLE();
	for (int i = 0; i < 32; i++) {
		HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_RESET);
		if (data & 0b10000000000000000000000000000000)
		{
			HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, GPIO_PIN_RESET);
		}
		//HAL_Delay(delay);
		HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_SET);
		data = data << 1;
	}
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_RESET);

}

uint32_t ADF_setfreq(float freq, float fPFD, uint8_t prescaler)
{
	float latch = freq / fPFD;
	uint32_t Nint = latch;
	latch = latch - Nint;
	uint32_t Nfrac = latch * 4096;
	uint32_t ret = 0;
	ret = ret | prescaler << 22;
	ret = ret | Nint << 14;
	ret = ret | Nfrac << 2;
	ret = ret | 0b00000001;
	return ret;
}


