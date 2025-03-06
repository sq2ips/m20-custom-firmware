/*
 *  lps22hb.c
 *  Based on https://github.com/KitSprout/KSDK/tree/master/firmwareSTM32/KSSTM_Module_LPS22HB/Program/modules
 *  Adapted by SQ2DK
 */

// This file is specifically made for M20 radiosondes. Uses specific hardware connections.
// file is basically customized KitSprout library  https://github.com/KitSprout/KSDK

#include <stdint.h>
#include "lps22hb.h"
//#include "math.h"   //only for calculating altitude

/**
  * @brief  SPI_RW
  */
uint8_t SPI_RW(uint8_t sendByte )
{
  SET_BIT(SPI1->CR1, SPI_CR1_SPE); 
	while(((SPI1->SR) & SPI_SR_TXE) != SPI_SR_TXE); // wait while tx-flag not empty
	*(uint8_t *)&(SPI1->DR) = sendByte; // write data to be transmitted to the SPI data register
	while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE); // wait while rx-buffer not empty
	/* Wait until the bus is ready before releasing Chip select */
	while(((SPI1->SR) & SPI_SR_BSY) == SPI_SR_BSY); 
  CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);

	return *(uint8_t *)&(SPI1->DR); // return received data from SPI data register
}

void LPS22_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  LL_GPIO_ResetOutputPin(LPS_CS_GPIO_Port, LPS_CS_Pin);
  SPI_RW(writeAddr);
  SPI_RW(writeData);
  LL_GPIO_SetOutputPin(LPS_CS_GPIO_Port, LPS_CS_Pin);
}

/**
 *  @brief  LPS22_ReadReg
 */
uint8_t LPS22_ReadReg( uint8_t readAddr )
{
  uint8_t readData;

  LL_GPIO_ResetOutputPin(LPS_CS_GPIO_Port, LPS_CS_Pin);
  SPI_RW(0x80 | readAddr);
  readData = SPI_RW(0x00);
  LL_GPIO_SetOutputPin(LPS_CS_GPIO_Port, LPS_CS_Pin);

  return readData;
}

//#define LPS22HB_InitRegNum    5
uint8_t LPS22_Init( void )
{
  uint8_t treg = 0;
//  uint8_t LPS22HB_InitData[LPS22HB_InitRegNum][2] = {
//    {0x00, LPS22HB_RES_CONF},       /* [0]  Normal mode (low-noise mode)  */
//    {0x04, MPU6500_PWR_MGMT_1},     /* [1]  Clock Source                  */
//    {0x10, MPU6500_INT_PIN_CFG},    /* [2]  Set INT_ANYRD_2CLEAR          */
//    {0x01, MPU6500_INT_ENABLE},     /* [3]  Set RAW_RDY_EN                */
//    {0x00, MPU6500_PWR_MGMT_2},     /* [4]  Enable Acc & Gyro             */
//    {0x00, MPU6500_SMPLRT_DIV},     /* [5]  Sample Rate Divider           */
//    {0x00, MPU6500_GYRO_CONFIG},    /* [6]  default : +-250dps            */
//    {0x00, MPU6500_ACCEL_CONFIG},   /* [7]  default : +-2G                */
//    {0x00, MPU6500_CONFIG},         /* [8]  default : GyrLPS_250Hz        */
//    {0x00, MPU6500_ACCEL_CONFIG_2}, /* [9]  default : AccLPS_460Hz        */
//    {0x30, MPU6500_USER_CTRL},      /* [10] Set I2C_MST_EN, I2C_IF_DIS    */
//  };

  if (LPS22_DeviceCheck() != SUCCESS) {
    return ERROR;
  }

  /* Normal mode (low-noise mode) */
  treg = LPS22_ReadReg(LPS22HB_RES_CONF);
  treg &= 0x02;
  treg |= 0x00;
  LPS22_WriteReg(LPS22HB_RES_CONF, treg);
  LL_mDelay(5);

  /* Control register 1 */
  treg  = 0x00;
  treg |= 0x50; // Output data rate, 75 Hz
  treg |= 0x00; // Low-pass filter disabled
  treg |= 0x02; // Block data update, enable
  LPS22_WriteReg(LPS22HB_CTRL_REG1, treg);
  LL_mDelay(5);

  /* Control register 2 */
  treg = LPS22_ReadReg(LPS22HB_CTRL_REG2);
  treg &= 0xED;
  treg |= 0x10;
  LPS22_WriteReg(LPS22HB_RES_CONF, treg);
  LL_mDelay(5);

  return SUCCESS;
}

/**
 *  @brief  LPS22_DeviceCheck
 */
uint8_t LPS22_DeviceCheck( void )
{
  uint8_t deviceID;

  deviceID = LPS22_ReadReg(LPS22HB_WHO_AM_I);

  if (deviceID != LPS22HB_DEVICE_ID) {
    return ERROR;
  }

  return SUCCESS;
}

/**
 *  @brief  LPS22_GetData
 */
float LPS22_GetPress( void )
{
  uint8_t buff[3];
  buff[0] = LPS22_ReadReg(LPS22HB_PRESS_OUT_XL);
  buff[1] = LPS22_ReadReg(LPS22HB_PRESS_OUT_L);
  buff[2] = LPS22_ReadReg(LPS22HB_PRESS_OUT_H);

  float press = (uint32_t)(((buff[2] & 0x7F)<<16 | buff[1]<<8 | buff[0]))/LPS22HB_SENS_HPA;
  return press;
}
float LPS22_GetTemp( void ){
  uint8_t buff[2];
    buff[0] = LPS22_ReadReg(LPS22HB_TEMP_OUT_L);
    buff[1] = LPS22_ReadReg(LPS22HB_TEMP_OUT_H);
    int16_t raw_temp = (int16_t)(buff[1]<<8 | buff[0]);
    float temp = raw_temp/LPS22HB_SENS_DEGC;
    return temp;
}


/**
 *  @brief  LPS22_GetAltitude
 */

/*
float LPS22_GetAltitude( float pressure )
{
  float altitude;

  // altitude above sea level (ASL)
  // (1.f - pow(*pressure / CONST_SEA_PRESSURE, CONST_PF)) * CONST_PF2
  // ((pow((1015.7 / *pressure), CONST_PF) - 1.0) * (25. + 273.15)) / 0.0065
  #define CONST_SEA_PRESSURE 102610.f
  #define CONST_PF 0.1902630958f //(1/5.25588f) Pressure factor
  #define CONST_PF2 44330.0f
  #define FIX_TEMP 25
  altitude = ((powf((1015.7f / pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;

  return altitude;
}
*/