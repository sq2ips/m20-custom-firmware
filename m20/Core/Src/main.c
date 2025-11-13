/* USER CODE BEGIN Header */
/*
 * Main file of M20 custom firmware project
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "utils.h"

#if GPS_TYPE == 1
#include "nmea.h"
#elif GPS_TYPE == 2
#include "xm_gps.h"
#endif
#include "adf.h"
#if HORUS_ENABLE
#include "fsk4.h"
#include "horus.h"
#endif
#if APRS_ENABLE
#include "afsk.h"
#include "aprs.h"
#endif
#if LPS22_ENABLE
#include "lps22hb.h"
#endif

#if DEBUG
#include <stdio.h>
#include <string.h>
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t GpsRxBuffer[GpsRxBuffer_SIZE];
uint16_t GpsBufferCounter = 0;
bool GpsBufferReady = false;

GPS GpsData;


#if GPS_WATCHDOG
struct GpsWatchdogStruct {
  bool PreviousFix;
  bool AfterRestart;
  uint8_t AfterRestartCounter;
  bool TriggerRestart;

#if GPS_TYPE == 2
  uint32_t LastTime;
#endif
} GpsWatchdog;
#endif

uint8_t GpsResetCount = 0;

#if LPS22_ENABLE
uint8_t lps_init;
#endif

int8_t LpsTemp = 0;
uint16_t LpsPress = 0; // *10

uint16_t BatVoltage = 0;

int16_t ExtTemp = 0; // *10

#if HORUS_ENABLE
HorusBinaryPacket HorusPacket;
#endif

#if APRS_ENABLE
APRSPacket AprsPacket;
#endif

#if APRS_ENABLE
char CodedBuffer[APRS_MAX_PACKET_LEN]; // Buffer for both HOURS and APRS frame, since APRS is always bigger, its size is used.
#else
char CodedBuffer[HORUS_CODED_SIZE]; // If only HOURS is used, use it's size.
#endif
uint8_t BufferLen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM22_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void GPS_Handler(void);
#if LED_MODE == 2
void LED_Handler(void);
#endif
void main_loop(void);
void DelayWithIWDG(uint16_t time);
#if GPS_TYPE == 1
void GpsAirborne(void);
#endif

#if DEBUG
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of
   * transmission */
  while (!LL_USART_IsActiveFlag_TXE(USART1)) {
  }
  LL_USART_TransmitData9(USART1, ch);

  return ch;
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void main_loop(void) {
  // LED
#if LED_MODE == 1
  LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif

  // LPS22HB sensor
#if LPS22_ENABLE
  if (lps_init == 0) {
    LpsTemp = (int8_t)Round(LPS22_GetTemp());
    LpsPress = (uint16_t)Round(LPS22_GetPress() * 10.0f);
  }
#if DEBUG
  printf("LPS22: Temp: %d C, press: %d /10 hPa\r\n", LpsTemp, LpsPress);
#endif
#endif

  // Bat voltage
#if BAT_ADC_ENABLE
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_8);
  LL_ADC_REG_StartConversion(ADC1);
  while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0) {}
  BatVoltage = LL_ADC_REG_ReadConversionData12(ADC1); // Raw ADC value 0-4095
  LL_ADC_ClearFlag_EOS(ADC1);
#if DEBUG
  printf("Bat voltage value: %d\r\n", BatVoltage);
#endif
#endif

  // NTC temp reading
#if NTC_ENABLE
  LL_GPIO_SetOutputPin(NTC_36K_GPIO_Port, NTC_36K_Pin);
  LL_mDelay(5);

  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_14);
  LL_ADC_REG_StartConversion(ADC1);
  while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0) {
  }
  uint16_t temp_adc_raw = LL_ADC_REG_ReadConversionData12(ADC1);
  LL_ADC_ClearFlag_EOS(ADC1);

  LL_GPIO_ResetOutputPin(NTC_36K_GPIO_Port, NTC_36K_Pin);

  // External temp calculating
  // Rntc = Vout * R1 /  Vin - Vout
  float NTC_R = ((temp_adc_raw * 36500) / (4096 - temp_adc_raw));
  float NTC_T = 1 / (-0.000400644 + (0.000490078 * Log(NTC_R)) +
                     (-0.000000720 * Log(NTC_R)*Log(NTC_R)*Log(NTC_R))) -
                273.15;
  ExtTemp = (int16_t)Round(NTC_T * 10.0);
#if DEBUG
  printf("NTC: raw: %d, Temp: %d /10 C\r\n", temp_adc_raw, ExtTemp);
#endif
#endif

#if GPS_WATCHDOG
// Set a flag if we have initial fix.
  if (GpsData.Fix > 1 && GpsData.Sats > 0) {
    GpsWatchdog.PreviousFix = true;
    GpsWatchdog.AfterRestart = false;
  }
  // Check GPS reset conditions only if there was initial fix.
  if (GpsWatchdog.PreviousFix) {
    if (GpsData.Fix > 1 && GpsData.Sats > 0) {
      // If there is  fix check spoofing specific conditions.
      // Ascent rate
      // printf("%d %d\r\n",(int32_t)GpsData.Time - TIME_PERIOD,
      // GpsWatchdog.LastTime);
      /*if ((int32_t)GpsData.AscentRate >= GPS_WATCHDOG_ASCENTRATE
          || (int32_t)GpsData.AscentRate <= (-1*GPS_WATCHDOG_ASCENTRATE)
          // Time deviation
          || (GpsWatchdog.LastTime != 0 &&
          (((int32_t)GpsData.Time - TIME_PERIOD) - GpsWatchdog.LastTime >
      GPS_WATCHDOG_MAX_D_TIME
          || GpsWatchdog.LastTime - ((int32_t)GpsData.Time - TIME_PERIOD) <
      GPS_WATCHDOG_MAX_D_TIME))) {
        // If conditions match trigger a restart
        GpsWatchdog.TriggerRestart = true;
      }

      GpsWatchdog.LastTime = GpsData.Time;
      // if (GpsData.AscentRate >= GPS_WATCHDOG_ASCENTRATE ||
      //(GpsWatchdog.LastTime != 0 &&
      // (GpsData.Time-TIME_PERIOD-GpsWatchdog.LastTime>GPS_WATCHDOG_MAX_TIME ||
      //
      GpsData.Time-TIME_PERIOD-GpsWatchdog.LastTime<(-1*GPS_WATCHDOG_MAX_TIME))))
      */
    }
    else if (GpsWatchdog.AfterRestart) {
      // If there is no fix and gps after a restart give it some time before
      // next restart
      GpsWatchdog.AfterRestartCounter++;
      if (GpsWatchdog.AfterRestartCounter >= GPS_WATCHDOG_ARC / TIME_PERIOD) {
        GpsWatchdog.TriggerRestart = true;
      }
    } else {
      GpsWatchdog.TriggerRestart = true;
    }
  }
  // printf("%d %d %d %d\r\n", GpsWatchdog.PreviousFix,
  //       GpsWatchdog.AfterRestart, GpsWatchdog.AfterRestartCounter,
  //       GpsWatchdog.TriggerRestart);

  if (GpsWatchdog.TriggerRestart) {
    LL_GPIO_ResetOutputPin(GPS_ON_GPIO_Port, GPS_ON_Pin); // disable GPS
    DelayWithIWDG(500);
    LL_GPIO_SetOutputPin(GPS_ON_GPIO_Port, GPS_ON_Pin); // enable GPS
  }
#endif

#if APRS_ENABLE
  AprsPacket.Hours = GpsData.Hours;
  AprsPacket.Minutes = GpsData.Minutes;
  AprsPacket.Seconds = GpsData.Seconds;
  AprsPacket.Lat = GpsData.Lat;
  AprsPacket.Lon = GpsData.Lon;
  AprsPacket.Speed = GpsData.Speed; // Doesn't work
  AprsPacket.Alt = GpsData.Alt;
  AprsPacket.Sats = GpsData.Sats;
  AprsPacket.GpsResetCount = GpsResetCount;
  AprsPacket.Temp = LpsTemp;
  AprsPacket.ExtTemp = ExtTemp;
  AprsPacket.Press = LpsPress;
  AprsPacket.BatVoltage = Round((BatVoltage*3300.0f)/4095);

  BufferLen = encode_APRS_packet(AprsPacket, CodedBuffer);

  AprsPacket.PacketCount++;

  // Transmit
  AFSK_start_TX(CodedBuffer, BufferLen);
  while (AFSK_Active) {DelayWithIWDG(10);}
#endif
#if HORUS_ENABLE && APRS_ENABLE
DelayWithIWDG(TX_PAUSE); // ???
#endif
#if HORUS_ENABLE
  HorusPacket.Hours = GpsData.Hours;
  HorusPacket.Minutes = GpsData.Minutes;
  HorusPacket.Seconds = GpsData.Seconds;
  HorusPacket.Lat = GpsData.Lat;
  HorusPacket.Lon = GpsData.Lon;
  HorusPacket.Speed = GpsData.Speed; // Doesn't work
  HorusPacket.Alt = GpsData.Alt;
  HorusPacket.Sats = GpsData.Sats;
  HorusPacket.AscentRate = GpsData.AscentRate;
  HorusPacket.Temp = LpsTemp;
  // ADC voltage: 3.3V, divided by ADC max value 4095 (2^12-1). That gives a
  // range between 0 for 0V and 1 for 3.3V. Than it's multiplied by max
  // variable value: 255, and divided by corresponding voltage: 5V, simplified
  // gives 187/4550 Note: this value will not go higher than 168 corresponding
  // to 3.3V max value of ADC
  HorusPacket.BatVoltage = (BatVoltage * 187) / 4550;
  HorusPacket.ExtTemp = ExtTemp;
  HorusPacket.Hum = 0; // Not implemented
  HorusPacket.Press = LpsPress;
  HorusPacket.GpsResetCount = GpsResetCount;

  // Horus checksum
  HorusPacket.Checksum =
      (uint16_t)crc16((char *)&HorusPacket, sizeof(HorusPacket) - 2);
  BufferLen = horus_l2_encode_tx_packet((unsigned char *)CodedBuffer,
                                            (unsigned char *)&HorusPacket,
                                            sizeof(HorusPacket));
  HorusPacket.PacketCount++;
  // Transmit
  FSK4_start_TX(CodedBuffer, BufferLen);
  while (FSK4_Active) {DelayWithIWDG(10);}
#endif

#if GPS_WATCHDOG
  if (GpsWatchdog.TriggerRestart) {
#if GPS_TYPE == 1
    GpsAirborne(); // Send a command to GPS module to change to airborne mode.
    // clear GPS buffer needed?
#endif
    // GPS type 2 doesn't need mode change

    // Reset variables to after restart state
    GpsWatchdog.TriggerRestart = false;
    GpsWatchdog.AfterRestart = true;
    GpsWatchdog.AfterRestartCounter = 0;
    //GpsWatchdog.LastTime = 0;

    GpsResetCount++;
  }
#endif

// LED
#if LED_MODE == 1
  LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 4);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM22_Init();
  MX_ADC_Init();
  MX_IWDG_Init();
  MX_TIM6_Init();
  MX_TIM21_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Power on modules
  LL_GPIO_SetOutputPin(POWER_ON_GPIO_Port, POWER_ON_Pin);
  LL_GPIO_SetOutputPin(GPS_ON_GPIO_Port, GPS_ON_Pin);
  LL_GPIO_SetOutputPin(RADIO_EN_GPIO_Port, RADIO_EN_Pin);
  adf_setup(); // Radio module setup

  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin); // LED ON

#if DEBUG
printf("Startup\r\n");
#endif

#if HORUS_ENABLE
  HorusPacket.PayloadID = HORUS_PAYLOAD_ID;
  HorusPacket.Unused = 32; // number in unused packet space, can be used to identify M20 transmitter durring flight, value is not important
#endif

  // Init of LPS22 sensor, try 5 times
#if LPS22_ENABLE
  for (uint8_t i = 0; i < 5; i++) {
    lps_init = LPS22_Init();
  if (lps_init == 0) break;
  }
#if DEBUG
  printf("LPS init: %d\r\n", lps_init);
#endif
#endif

  // ADC init
#if NTC_ENABLE || BAT_ADC_ENABLE
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) {}
#endif

  // GPS UART init
  LL_LPUART_Enable(LPUART1); // Disable interrupt for sending command
  LL_LPUART_EnableIT_RXNE(LPUART1);

#if GPS_TYPE == 1
  // u-blox change mode to airborne
  DelayWithIWDG(2000); // Wait for full GPS start
  GpsAirborne(); // Send a command to GPS module to change to airborne mode.
  DelayWithIWDG(100);
#endif

  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin); // LED OFF

  // main loop timer
  LL_TIM_EnableCounter(TIM22);
  LL_TIM_EnableIT_UPDATE(TIM22);

  // LED timer
#if LED_MODE == 2
  LL_TIM_EnableCounter(TIM6);
  LL_TIM_EnableIT_UPDATE(TIM6);
#endif

  /* Interrupt priorites:
   * TIM2 - 4FSK modulation timer: 0
   * TIM21 - AFSK modulation timer: 0
   * LPUART1 - GPS UART RX: 1
   * TIM6 - LED timer: 2
   * SysTick: 3
   * TIM22 - main loop: 4
   */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LL_IWDG_ReloadCounter(IWDG);
    if (GpsBufferReady) {
      #if GPS_TYPE == 1
      ParseNMEA(&GpsData, GpsRxBuffer);
      #elif GPS_TYPE == 2
      parseXM(&GpsData, GpsRxBuffer);
      #endif
      #if GPS_DEBUG
      gps_debug(GpsData);
      #endif
      GpsBufferReady = false;
    }
    LL_mDelay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_3, LL_RCC_PLL_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }

  LL_Init1msTick(12000000);

  LL_SetSystemCoreClock(12000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_SYSCLK);
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_1);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**ADC GPIO Configuration
  PC0   ------> ADC_IN10
  PC4   ------> ADC_IN14
  PB0   ------> ADC_IN8
  */
  GPIO_InitStruct.Pin = PAYLOAD_ADC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PAYLOAD_ADC_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NTC_ADC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NTC_ADC_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BAT_ADC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAT_ADC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_8);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_10);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_14);

  /** Common config
  */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_3CYCLES_5);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /* USER CODE BEGIN ADC_Init 2 */
  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  LL_IWDG_Enable(IWDG);
  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_8);
  LL_IWDG_SetReloadCounter(IWDG, 4095);
  while (LL_IWDG_IsReady(IWDG) != 1)
  {
  }

  LL_IWDG_ReloadCounter(IWDG);
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  /**LPUART1 GPIO Configuration
  PC10   ------> LPUART1_TX
  PC11   ------> LPUART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* LPUART1 interrupt Init */
  NVIC_SetPriority(LPUART1_IRQn, 1);
  NVIC_EnableIRQ(LPUART1_IRQn);

  /* USER CODE BEGIN LPUART1_Init 1 */
#if GPS_TYPE == 1
  LPUART_InitStruct.BaudRate = 9600;
#elif GPS_TYPE == 2
  LPUART_InitStruct.BaudRate = 38400;
#endif
  /* USER CODE END LPUART1_Init 1 */
  // LPUART_InitStruct.BaudRate = 9600;
  LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
  LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
  LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
  LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
  LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
  LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_ENABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* TIM6 interrupt Init */
  NVIC_SetPriority(TIM6_DAC_IRQn, 2);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */
  TIM_InitStruct.Autoreload = ((LED_PERIOD * 1000) / 5) - 1;
  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 60000;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  // TIM_InitStruct.Autoreload = 65535;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM21);

  /* TIM21 interrupt Init */
  NVIC_SetPriority(TIM21_IRQn, 0);
  NVIC_EnableIRQ(TIM21_IRQn);

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM21, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM21);
  LL_TIM_OC_EnablePreload(TIM21, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM21, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM21, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM21, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM21);
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**TIM21 GPIO Configuration
  PB13   ------> TIM21_CH1
  */
  GPIO_InitStruct.Pin = ADF_TX_Data_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(ADF_TX_Data_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */
  /* USER CODE END TIM22_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM22);

  /* TIM22 interrupt Init */
  NVIC_SetPriority(TIM22_IRQn, 3);
  NVIC_EnableIRQ(TIM22_IRQn);

  /* USER CODE BEGIN TIM22_Init 1 */
  TIM_InitStruct.Autoreload = ((TIME_PERIOD * 1000) / 5) - 1;
  /* USER CODE END TIM22_Init 1 */
  TIM_InitStruct.Prescaler = 60000;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  // TIM_InitStruct.Autoreload = 2400;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM22, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM22);
  LL_TIM_SetClockSource(TIM22, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM22, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM22);
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOH);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LPS_CS_GPIO_Port, LPS_CS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(RF_Boost_GPIO_Port, RF_Boost_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPS_ON_GPIO_Port, GPS_ON_Pin);

  /**/
  LL_GPIO_ResetOutputPin(RADIO_EN_GPIO_Port, RADIO_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(ADF_CLK_GPIO_Port, ADF_CLK_Pin);

  /**/
  LL_GPIO_ResetOutputPin(ADF_Data_GPIO_Port, ADF_Data_Pin);

  /**/
  LL_GPIO_ResetOutputPin(ADF_LE_GPIO_Port, ADF_LE_Pin);

  /**/
  LL_GPIO_ResetOutputPin(POWER_ON_GPIO_Port, POWER_ON_Pin);

  /**/
  LL_GPIO_ResetOutputPin(ADF_CE_GPIO_Port, ADF_CE_Pin);

  /**/
  LL_GPIO_SetOutputPin(NTC_475K_GPIO_Port, NTC_475K_Pin);

  /**/
  LL_GPIO_ResetOutputPin(NTC_36K_GPIO_Port, NTC_36K_Pin);

  /**/
  LL_GPIO_SetOutputPin(NTC_12K_GPIO_Port, NTC_12K_Pin);

  /**/
  LL_GPIO_SetOutputPin(NTC_2M_GPIO_Port, NTC_2M_Pin);

  /**/
  LL_GPIO_SetOutputPin(NTC_330K_GPIO_Port, NTC_330K_Pin);

  /**/
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LPS_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LPS_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RF_Boost_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RF_Boost_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = GPS_ON_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPS_ON_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RADIO_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RADIO_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ADF_CLK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADF_CLK_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ADF_Data_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADF_Data_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ADF_LE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADF_LE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = POWER_ON_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(POWER_ON_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ADF_CE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADF_CE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NTC_475K_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NTC_475K_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NTC_36K_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NTC_36K_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NTC_12K_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NTC_12K_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NTC_2M_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NTC_2M_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NTC_330K_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NTC_330K_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void GPS_Handler(void) {
  if (LL_LPUART_IsEnabledIT_RXNE(LPUART1) &&
      LL_LPUART_IsActiveFlag_RXNE(LPUART1)) {
    if (GpsBufferCounter >= GpsRxBuffer_SIZE) {
      GpsBufferCounter = 0;
      GpsBufferReady = true;
    }
    GpsRxBuffer[GpsBufferCounter] = LL_LPUART_ReceiveData8(LPUART1);
    GpsBufferCounter++;
  } else if (LL_LPUART_IsActiveFlag_ORE(LPUART1)) {
    LL_LPUART_ClearFlag_ORE(LPUART1);
#if DEBUG
    printf("ORE!\r\n");
#endif
  }
}
#if LED_MODE == 2
void LED_Handler(void) {
  uint8_t fix = GpsData.Fix;
  uint16_t alt = GpsData.Alt;
  if (LED_DISABLE_ALT != 0 && alt >= LED_DISABLE_ALT) {
    LL_TIM_DisableCounter(TIM6);
    LL_TIM_DisableIT_UPDATE(TIM6);
  }
  for (; fix > 0; fix--) {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    LL_mDelay(LED_MODE_2_BLINK_TIME);
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
    LL_mDelay(LED_MODE_2_BLINK_PAUSE);
  }
}
#endif
void DelayWithIWDG(uint16_t time) {
  for (uint16_t i = 0; i < time / 10; i++) {
    LL_IWDG_ReloadCounter(IWDG);
    LL_mDelay(10);
  }
}
#if GPS_TYPE == 1
void GpsAirborne(void) {
  LL_LPUART_DisableIT_RXNE(LPUART1);  // disable UART RX interrupt to not occure while mode change
  for (uint8_t ih = 0; ih < 2; ih++) {
    for (uint8_t ig = 0; ig < 44; ig++) {
      while (!LL_LPUART_IsActiveFlag_TXE(LPUART1)) {
      }
      LL_LPUART_TransmitData8(LPUART1, GPS_airborne[ig]);
    }
    while (!LL_LPUART_IsActiveFlag_TC(LPUART1)) {
    }
    if (ih == 0)
      DelayWithIWDG(900);
  }
  LL_LPUART_EnableIT_RXNE(LPUART1); // reenable UART RX interrupt
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
