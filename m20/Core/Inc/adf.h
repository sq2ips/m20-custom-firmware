#ifndef ADF_H_
#define ADF_H_

#include <stdint.h>

void adf_reset_config(void);
void adf_reset_register_zero(void);
void adf_reset_register_one(void);
void adf_reset_register_two(void);
void adf_reset_register_three(void);
void adf_reset(void);
void adf_turn_off(void);
void adf_write_config(void);
void adf_write_register_zero(void);
void adf_write_register_one(void);
void adf_write_register_two(void);
void adf_write_register_three(void);
void adf_write_register(uint32_t data);
void adf_set_frequency_error_correction(uint16_t error);
void adf_set_r_divider(uint8_t r);
void adf_set_vco_adjust(uint8_t adjust);
void adf_set_frequency(float freq);
void adf_4fsk_fone(uint8_t tone);
void adf_set_n(uint8_t n);
void adf_set_m(uint16_t m);
void adf_set_pa_level(uint8_t level);
void adf_set_pll_enable(uint8_t enable);
void adf_set_pa_enable(uint8_t enable);
void adf_setup(void);
void adf_RF_on(float freq, uint8_t power);
void adf_RF_off(void);
void adf_set_deviation(uint16_t modulation_deviation);

// General Purpose ===========================================================
#define ADF_OFF 0
#define ADF_ON 1

// Register Constants ========================================================
// Register 1 ----------------------------------------------------------------
#define ADF_PRESCALER_4_5 0
#define ADF_PRESCALER_8_9 1

// Register 2 ----------------------------------------------------------------
#define ADF_MODULATION_FSK 0
#define ADF_MODULATION_GFSK 1
#define ADF_MODULATION_ASK 2
#define ADF_MODULATION_OOK 3

// Register 3 ----------------------------------------------------------------
#define ADF_CP_CURRENT_0_3 0
#define ADF_CP_CURRENT_0_9 1
#define ADF_CP_CURRENT_1_5 2
#define ADF_CP_CURRENT_2_1 3
#define ADF_MUXOUT_LOGIC_LOW 0
#define ADF_MUXOUT_LOGIC_HIGH 1
#define ADF_MUXOUT_REG_READY 3
#define ADF_MUXOUT_DIGITAL_LOCK 4
#define ADF_MUXOUT_ANALOGUE_LOCK 5
#define ADF_MUXOUT_R_DIVIDER_2 1
#define ADF_MUXOUT_N_DIVIDER_2 7
#define ADF_MUXOUT_RF_R_DIVIDER 8
#define ADF_MUXOUT_DATA_RATE 9
#define ADF_MUXOUT_BATT_2_35 10
#define ADF_MUXOUT_BATT_2_75 11
#define ADF_MUXOUT_BATT_3 12
#define ADF_MUXOUT_BATT_3_25 13
#define ADF_MUXOUT_TEST_MODE 14
#define ADF_MUXOUT_SD_TEST_MODE 15
#define ADF_LD_PRECISION_3_CYCLES 0
#define ADF_LD_PRECISION_5_CYCLES 1

#define ADF_CLOCK 8000000       // Clock speed of adf7012 chip coming from STM32 (in Hz) (set to HSE 8MHz oscilator)

#endif
