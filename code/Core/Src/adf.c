// based on https://github.com/adamgreig/wombat
// adopted by SQ2DK

#include <stdio.h>
#include <main.h>
#include "adf.h"
#include "config.h"

// Configuration storage structs =============================================
struct {
    struct {
        uint16_t frequency_error_correction;
        uint8_t r_divider;
        uint8_t crystal_doubler;
        uint8_t crystal_oscillator_disable;
        uint8_t clock_out_divider;
        uint8_t vco_adjust;
        uint8_t output_divider;
    } r0;

    struct {
        uint16_t fractional_n;
        uint8_t integer_n;
        uint8_t prescaler;
    } r1;

    struct {
        uint8_t mod_control;
        uint8_t gook;
        uint8_t power_amplifier_level;
        uint16_t modulation_deviation;
        uint8_t gfsk_modulation_control;
        uint8_t index_counter;
    } r2;

    struct {
        uint8_t pll_enable;
        uint8_t pa_enable;
        uint8_t clkout_enable;
        uint8_t data_invert;
        uint8_t charge_pump_current;
        uint8_t bleed_up;
        uint8_t bleed_down;
        uint8_t vco_disable;
        uint8_t muxout;
        uint8_t ld_precision;
        uint8_t vco_bias;
        uint8_t pa_bias;
        uint8_t pll_test_mode;
        uint8_t sd_test_mode;
    } r3;
} adf_config;

// Prototypes for internal functions =========================================

void adf_reset_register_zero(void);
void adf_reset_register_one(void);
void adf_reset_register_two(void);
void adf_reset_register_three(void);
void adf_reset(void);
void adf_write_register_zero(void);
void adf_write_register_one(void);
void adf_write_register_two(void);
void adf_write_register_three(void);
void adf_write_register(uint32_t reg);



// Configuration functions ===================================================

// Config resetting functions --------------------------------------------
void adf_reset_config(void) {
    adf_reset_register_zero();
    adf_reset_register_one();
    adf_reset_register_two();
    adf_reset_register_three();
    adf_reset();

    //HAL_Delay(10);      //as pin for reading adf ready state is not available, we will wait a moment


}

/* Bellow values are based on experiments. Do not change if not required. Some of those are critical and are not documented
 * vco_adjust=3; pa_bias = 7; vco_bias = 0; crystal_oscillator_disable = 1;
 * For modulation deviation set value on experimental basis, for FM channel width to be appropriate. In theory in 270z steps?
 */

void adf_reset_register_zero(void) {
    adf_config.r0.frequency_error_correction = ADF_FREQ_CORRECTION;   //11Bit Freq err corr 0b10011 - or whatever reason M20
    adf_config.r0.r_divider = 1;
    adf_config.r0.crystal_doubler = ADF_OFF;
    adf_config.r0.crystal_oscillator_disable = 1;  //osclator disabled - we are getting clock from STM
    adf_config.r0.clock_out_divider = 8;        // 0b1000 = 8 FOR M20
    adf_config.r0.vco_adjust = 3;                //maximum VCO adjust b11
    adf_config.r0.output_divider = 1;            // for M20
}

void adf_reset_register_one(void) {
    adf_config.r1.fractional_n = 0;
    adf_config.r1.integer_n = 0;
    adf_config.r1.prescaler = ADF_PRESCALER_4_5;
}

void adf_reset_register_two(void) {
    adf_config.r2.mod_control = ADF_MODULATION_FSK;
    adf_config.r2.gook = ADF_OFF;
    adf_config.r2.power_amplifier_level = 0;  // power level
    adf_config.r2.modulation_deviation = 2;    //5= about 5k5Hz, 10=11kHz
    //adf_config.r2.modulation_deviation = 11;
    adf_config.r2.gfsk_modulation_control = 0;
    adf_config.r2.index_counter = 0;
}

void adf_reset_register_three(void) {
    adf_config.r3.pll_enable = ADF_OFF;
    adf_config.r3.pa_enable = ADF_OFF;
    adf_config.r3.clkout_enable = ADF_OFF;
    adf_config.r3.data_invert = ADF_ON;
    adf_config.r3.charge_pump_current = ADF_CP_CURRENT_1_5;
    adf_config.r3.bleed_up = ADF_OFF;
    adf_config.r3.bleed_down = ADF_OFF;
    adf_config.r3.vco_disable = ADF_OFF;
    adf_config.r3.muxout = ADF_MUXOUT_REG_READY;
    adf_config.r3.ld_precision = ADF_LD_PRECISION_3_CYCLES;
    adf_config.r3.vco_bias = 0;   //was 4
    adf_config.r3.pa_bias = 7;
    adf_config.r3.pll_test_mode = 0;
    adf_config.r3.sd_test_mode = 0;
}

void adf_reset(void) {
	HAL_GPIO_WritePin(ADF_CE_GPIO_Port, ADF_CE_Pin, 0);
	HAL_GPIO_WritePin(RADIO_EN_GPIO_Port, RADIO_EN_Pin, 0);
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, 0);
    HAL_GPIO_WritePin(ADF_TX_Data_GPIO_Port, ADF_TX_Data_Pin, 1);
    HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, 1);
    HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, 1);
    HAL_Delay(1); //short delay between powering off and on
    HAL_GPIO_WritePin(RADIO_EN_GPIO_Port, RADIO_EN_Pin, 1);
    HAL_GPIO_WritePin(ADF_CE_GPIO_Port, ADF_CE_Pin, 1);
   // HAL_Delay(100);
}

void adf_turn_off(void) {
	HAL_GPIO_WritePin(RADIO_EN_GPIO_Port, RADIO_EN_Pin, 0);  //TODO - check power consumption. Check if registry is gone.
    HAL_GPIO_WritePin(ADF_CE_GPIO_Port, ADF_CE_Pin, 0);
}

// Configuration writing functions ---------------------------------------

void adf_write_config(void) {
    adf_write_register_zero();
    adf_write_register_one();
    adf_write_register_two();
    adf_write_register_three();
}

void adf_write_register_zero(void) {
    uint32_t reg =
        (0) |
        ((adf_config.r0.frequency_error_correction  & 0x7FF) <<  2) |
        ((adf_config.r0.r_divider                   & 0xF  ) << 13) |
        ((adf_config.r0.crystal_doubler             & 0x1  ) << 17) |
        ((adf_config.r0.crystal_oscillator_disable  & 0x1  ) << 18) |
        ((adf_config.r0.clock_out_divider           & 0xF  ) << 19) |
        ((adf_config.r0.vco_adjust                  & 0x3  ) << 23) |
        ((adf_config.r0.output_divider              & 0x3  ) << 25);
    adf_write_register(reg);

}

void adf_write_register_one(void) {
    uint32_t reg =
        (1) |
        ((adf_config.r1.fractional_n                & 0xFFF) <<  2) |
        ((adf_config.r1.integer_n                   & 0xFF ) << 14) |
        ((adf_config.r1.prescaler                   & 0x1  ) << 22);
     adf_write_register(reg);

}

void adf_write_register_two(void) {
    uint32_t reg =
        (2) |
        ((adf_config.r2.mod_control                 & 0x3  ) <<  2) |
        ((adf_config.r2.gook                        & 0x1  ) <<  4) |
        ((adf_config.r2.power_amplifier_level       & 0x3F ) <<  5) |
        ((adf_config.r2.modulation_deviation        & 0x1FF) << 11) |
        ((adf_config.r2.gfsk_modulation_control     & 0x7  ) << 20) |
        ((adf_config.r2.index_counter               & 0x3  ) << 23);
    adf_write_register(reg);

}

void adf_write_register_three(void) {
    uint32_t reg =
        (3) |
        ((adf_config.r3.pll_enable                  & 0x1  ) <<  2) |
        ((adf_config.r3.pa_enable                   & 0x1  ) <<  3) |
        ((adf_config.r3.clkout_enable               & 0x1  ) <<  4) |
        ((adf_config.r3.data_invert                 & 0x1  ) <<  5) |
        ((adf_config.r3.charge_pump_current         & 0x3  ) <<  6) |
        ((adf_config.r3.bleed_up                    & 0x1  ) <<  8) |
        ((adf_config.r3.bleed_down                  & 0x1  ) <<  9) |
        ((adf_config.r3.vco_disable                 & 0x1  ) << 10) |
        ((adf_config.r3.muxout                      & 0xF  ) << 11) |
        ((adf_config.r3.ld_precision                & 0x1  ) << 15) |
        ((adf_config.r3.vco_bias                    & 0xF  ) << 16) |
        ((adf_config.r3.pa_bias                     & 0x7  ) << 20) |
        ((adf_config.r3.pll_test_mode               & 0x1F ) << 23) |
        ((adf_config.r3.sd_test_mode                & 0xF  ) << 28);
    adf_write_register(reg);


}

void adf_write_register(uint32_t data)
{
	/*
	 * Below procedure is not the prettiest one, but it works. Lots of work was put into it, so be careful when changing.
	 * wl value might be changed - delay is implemented, since other ways did not worked. It is possible, that they are not
	 * required, or that they would have to be adjusted if CPU frequency would be adjusted.
	 */

	uint16_t wl=5;  //wl=10;
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_SET);

	for(uint16_t n=0; n< (wl*2); n++)
	    			asm("NOP");

	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_RESET);

	for(uint16_t n=0; n< wl; n++)
		    			asm("NOP");


	for (int i = 0; i < 32; i++) {
		HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_RESET);
		if (data & 0b10000000000000000000000000000000)
		{
			HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(ADF_Data_GPIO_Port, ADF_Data_Pin, GPIO_PIN_RESET);
		}

		for(uint16_t n=0; n< (wl *2); n++)
			    			asm("NOP");

		HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_SET);
		data = data << 1;
	}

	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_SET);
	for(uint16_t n=0; n< (wl*2); n++)
		    			asm("NOP");

	HAL_GPIO_WritePin(ADF_CLK_GPIO_Port, ADF_CLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_RESET);


}

// Configuration setting functions ---------------------------------------
void adf_set_frequency_error_correction(uint16_t error) {
    adf_config.r0.frequency_error_correction = error;
}

void adf_set_r_divider(uint8_t r) {
    adf_config.r0.r_divider = r;
}

void adf_set_vco_adjust(uint8_t adjust) {
    adf_config.r0.vco_adjust = adjust;
}




void adf_set_frequency(float freq)
{

	float latch = freq / ADF_CLOCK;   //changed for 9MHz input
	uint32_t Nint = latch;
	latch = latch - Nint;
	uint32_t Nfrac = latch * 4096;
	adf_set_n(Nint);
	adf_set_m((uint16_t)Nfrac);

}

void adf_4fsk_fone(uint8_t tone){
/* minimum frequency change is defined as Fpfd/(2^15). Fpfd=osc/Rdiv (Rdiv=1 in our case).
 * Thus for achieving 270Hz frequency step we have to provide 9MHz osc signal
 * using frequency adjust register is fastest, since one have to write only register 0
 * (consecutive registers does not need to be sent)
 * Frequency shifts smaller than calculated cannot be achieved, thus baudrate slower than 100bd
 * are not possible. Only multiplications of 100bd are alowed with standard tone spacings
 */

	uint16_t freq_corr_regist_tmp = adf_config.r0.frequency_error_correction;  // Temporarily store correction value, as we are going to change it
	adf_config.r0.frequency_error_correction = adf_config.r0.frequency_error_correction + tone; //add tone space to current freq corr.
	adf_write_register_zero();                                         //write reg0. no other registers have to be changed.
	adf_config.r0.frequency_error_correction = freq_corr_regist_tmp;  // restore original value


}

void adf_set_n(uint8_t n) {
    adf_config.r1.integer_n = n;
}

void adf_set_m(uint16_t m) {
    adf_config.r1.fractional_n = m;
}

void adf_set_pa_level(uint8_t level) {
    adf_config.r2.power_amplifier_level = level;
}

void adf_set_pll_enable(uint8_t enable) {
    adf_config.r3.pll_enable = enable;
}

void adf_set_pa_enable(uint8_t enable) {
    adf_config.r3.pa_enable = enable;
}

void adf_setup(void) {
	HAL_GPIO_WritePin(ADF_CE_GPIO_Port, ADF_CE_Pin, 1);
    adf_reset_config();
    adf_set_r_divider(1);            // We will be using whole 9(or 8)MHz for freq reference for adf7012
    adf_set_frequency(QRG_CW);       // Temporarily set freq for CW - it is going to be changed later when turning on TX
    adf_set_pll_enable(ADF_ON);
    adf_write_config();
}


void adf_RF_on(float freq, uint8_t power) {  //arguments are TX frequency (Hz) and power level (0-63)
	HAL_GPIO_WritePin(ADF_CE_GPIO_Port, ADF_CE_Pin, 1);
	adf_set_frequency(freq);
	adf_set_pa_enable(ADF_ON);
    adf_set_pa_level(power);
    adf_write_config();
    HAL_GPIO_WritePin(RF_Boost_GPIO_Port, RF_Boost_Pin, !RF_BOOST_ACTIVE);  //RF boost 0 - is ON
    //adf_write_register_three();
}

void adf_RF_off(void) {
    adf_set_pa_enable(ADF_OFF);
    adf_set_pa_level(0);
    adf_write_config();
    HAL_GPIO_WritePin(RF_Boost_GPIO_Port, RF_Boost_Pin, 1); //boost pin high for RF power off. Reduces power by ~15dB
}



//MUX pin is not connected in M20

