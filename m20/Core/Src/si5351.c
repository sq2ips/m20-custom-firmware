/*
 *  Si5351 module I2C driver
 *  by SQ2IPS
 */
#include "si5351.h"
#include "main.h"

enum {
    SI5351_REG0_DEVICE_STATUS                       = 0,
    SI5351_REG1_INTERRUPT_STATUS_STICKY             = 1,
    SI5351_REG2_INTERRUPT_STATUS_MASK               = 2,
    SI5351_REG3_OUTPUT_ENABLE_CONTROL               = 3,
    SI5351_REG9_OEB_PIN_ENABLE_CONTROL              = 9,
    SI5351_REG15_PLL_INPUT_SOURCE                   = 15,
    SI5351_REG16_CLK0_CONTROL                       = 16,
    SI5351_REG17_CLK1_CONTROL                       = 17,
    SI5351_REG18_CLK2_CONTROL                       = 18,
    SI5351_REG19_CLK3_CONTROL                       = 19,
    SI5351_REG20_CLK4_CONTROL                       = 20,
    SI5351_REG21_CLK5_CONTROL                       = 21,
    SI5351_REG22_CLK6_CONTROL                       = 22,
    SI5351_REG23_CLK7_CONTROL                       = 23,
    SI5351_REG24_CLK3_0_DISABLE_STATE               = 24,
    SI5351_REG25_CLK7_4_DISABLE_STATE               = 25,

    SI5351_REG42_MULTISYNTH0_PARAM_3_MED       = 42,
    SI5351_REG43_MULTISYNTH0_PARAM_3_LOW        = 43,
    SI5351_REG44_MULTISYNTH0_PARAM_1_HIGH_R0    = 44,
    SI5351_REG45_MULTISYNTH0_PARAM_1_MED           = 45,
    SI5351_REG46_MULTISYNTH0_PARAM_1_LOW           = 46,
    SI5351_REG47_MULTISYNTH0_PARAM_3_2_HIGH           = 47,
    SI5351_REG48_MULTISYNTH0_PARAM_2_MED           = 48,
    SI5351_REG49_MULTISYNTH0_PARAM_2_LOW           = 49,

    SI5351_REG50_MULTISYNTH1_PARAM_1           = 50,
    SI5351_REG51_MULTISYNTH1_PARAM_2           = 51,
    SI5351_REG52_MULTISYNTH1_PARAM_3           = 52,
    SI5351_REG53_MULTISYNTH1_PARAM_4           = 53,
    SI5351_REG54_MULTISYNTH1_PARAM_5           = 54,
    SI5351_REG55_MULTISYNTH1_PARAM_6           = 55,
    SI5351_REG56_MULTISYNTH1_PARAM_7           = 56,
    SI5351_REG57_MULTISYNTH1_PARAM_8           = 57,
    SI5351_REG58_MULTISYNTH2_PARAM_1           = 58,
    SI5351_REG59_MULTISYNTH2_PARAM_2           = 59,
    SI5351_REG60_MULTISYNTH2_PARAM_3           = 60,
    SI5351_REG61_MULTISYNTH2_PARAM_4           = 61,
    SI5351_REG62_MULTISYNTH2_PARAM_5           = 62,
    SI5351_REG63_MULTISYNTH2_PARAM_6           = 63,
    SI5351_REG64_MULTISYNTH2_PARAM_7           = 64,
    SI5351_REG65_MULTISYNTH2_PARAM_8           = 65,
    SI5351_REG66_MULTISYNTH3_PARAM_1           = 66,
    SI5351_REG67_MULTISYNTH3_PARAM_2           = 67,
    SI5351_REG68_MULTISYNTH3_PARAM_3           = 68,
    SI5351_REG69_MULTISYNTH3_PARAM_4           = 69,
    SI5351_REG70_MULTISYNTH3_PARAM_5           = 70,
    SI5351_REG71_MULTISYNTH3_PARAM_6           = 71,
    SI5351_REG72_MULTISYNTH3_PARAM_7           = 72,
    SI5351_REG73_MULTISYNTH3_PARAM_8           = 73,
    SI5351_REG74_MULTISYNTH4_PARAM_1           = 74,
    SI5351_REG75_MULTISYNTH4_PARAM_2           = 75,
    SI5351_REG76_MULTISYNTH4_PARAM_3           = 76,
    SI5351_REG77_MULTISYNTH4_PARAM_4           = 77,
    SI5351_REG78_MULTISYNTH4_PARAM_5           = 78,
    SI5351_REG79_MULTISYNTH4_PARAM_6           = 79,
    SI5351_REG80_MULTISYNTH4_PARAM_7           = 80,
    SI5351_REG81_MULTISYNTH4_PARAM_8           = 81,
    SI5351_REG82_MULTISYNTH5_PARAM_1           = 82,
    SI5351_REG83_MULTISYNTH5_PARAM_2           = 83,
    SI5351_REG84_MULTISYNTH5_PARAM_3           = 84,
    SI5351_REG85_MULTISYNTH5_PARAM_4           = 85,
    SI5351_REG86_MULTISYNTH5_PARAM_5           = 86,
    SI5351_REG87_MULTISYNTH5_PARAM_6           = 87,
    SI5351_REG88_MULTISYNTH5_PARAM_7           = 88,
    SI5351_REG89_MULTISYNTH5_PARAM_8           = 89,
    SI5351_REG90_MULTISYNTH6_PARAM             = 90,
    SI5351_REG91_MULTISYNTH7_PARAM             = 91,
    SI5351_REG92_CLOCK_6_7_OUTPUT_DIVIDER           = 92,
    SI5351_REG165_CLK0_INITIAL_PHASE_OFFSET         = 165,
    SI5351_REG166_CLK1_INITIAL_PHASE_OFFSET         = 166,
    SI5351_REG167_CLK2_INITIAL_PHASE_OFFSET         = 167,
    SI5351_REG168_CLK3_INITIAL_PHASE_OFFSET         = 168,
    SI5351_REG169_CLK4_INITIAL_PHASE_OFFSET         = 169,
    SI5351_REG170_CLK5_INITIAL_PHASE_OFFSET         = 170,
    SI5351_REG177_PLL_RESET                         = 177,
    SI5351_REG183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183
};

void si5351_write(uint8_t reg, uint8_t val, uint8_t len){
    // Wait until I2C is not busy
    while (LL_I2C_IsActiveFlag_BUSY(I2C2));

    // Start condition + slave address (write)
    LL_I2C_HandleTransfer(I2C2,
                          SI5351_ADDR<<1,
                          LL_I2C_ADDRSLAVE_7BIT,
                          len+1, // number of bytes (register + data)
                          LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_WRITE);

    // Wait TXIS (Transmit interrupt status)
    while (!LL_I2C_IsActiveFlag_TXIS(I2C2));
    LL_I2C_TransmitData8(I2C2, reg);

    for(uint8_t i = 0; i<len; i++){
        while (!LL_I2C_IsActiveFlag_TXIS(I2C2)){}
        LL_I2C_TransmitData8(I2C2, val);
    }
    
    // Wait for STOP condition
    while (!LL_I2C_IsActiveFlag_STOP(I2C2));

    // Clear STOP flag
    LL_I2C_ClearFlag_STOP(I2C2);
}

void si5351_set_multisynth(){
    uint8_t r0_div = 0;
    uint32_t ms0_p1 = 9728;
    uint32_t ms0_p2 = 0;
    uint32_t ms0_p3 = 1;

    si5351_write(SI5351_REG42_MULTISYNTH0_PARAM_3_MED, (ms0_p3>>8) & 0xFF, 1);
    si5351_write(SI5351_REG43_MULTISYNTH0_PARAM_3_LOW, ms0_p3 & 0xFF, 1);
    si5351_write(SI5351_REG44_MULTISYNTH0_PARAM_1_HIGH_R0, ((r0_div&7) << 4) | ((ms0_p1>>16) & 3), 1);
    si5351_write(SI5351_REG45_MULTISYNTH0_PARAM_1_MED, (ms0_p1>>8) & 0xFF, 1);
    si5351_write(SI5351_REG46_MULTISYNTH0_PARAM_1_LOW, ms0_p1 & 0xFF, 1);
    si5351_write(SI5351_REG47_MULTISYNTH0_PARAM_3_2_HIGH, (((ms0_p3 >> 16) & 0x0F) << 4) | ((ms0_p2 >> 16) & 0x0F), 1);
    si5351_write(SI5351_REG48_MULTISYNTH0_PARAM_2_MED, (ms0_p2>>8) & 0xFF, 1);
    si5351_write(SI5351_REG49_MULTISYNTH0_PARAM_2_LOW, ms0_p2 & 0xFF, 1);

}

void si5351_init(){
    // disable all outputs
    uint8_t val = 0xFF;
    si5351_write(SI5351_REG3_OUTPUT_ENABLE_CONTROL, val, 1);

    // powerdown all output drivers
    val = 0x80;
    si5351_write(SI5351_REG16_CLK0_CONTROL, val, 8);

    // Crystal
    val = (3<<6);
    si5351_write(SI5351_REG183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, val, 1);

    // no masking
    val = 0x00;
    si5351_write(SI5351_REG2_INTERRUPT_STATUS_MASK, val, 1);

    // PLL input source ???
    val = 0x00;
    si5351_write(SI5351_REG15_PLL_INPUT_SOURCE, val, 1);

    // configure CLK0, power up, fraction mode, PLLA, not inverted, MultiSync0 source, 8 mA
    val = 0b00001111;
    si5351_write(SI5351_REG16_CLK0_CONTROL, val, 1);

    // disable state LOW
    val = 0x00;
    si5351_write(SI5351_REG24_CLK3_0_DISABLE_STATE, val, 1);

    // Multisynth
    si5351_set_multisynth();

    // PLL soft reset
    val = 0xAC;
    si5351_write(SI5351_REG177_PLL_RESET, val, 1);

    // enable output 0
    val = 0b11111110;
    si5351_write(SI5351_REG3_OUTPUT_ENABLE_CONTROL, val, 1);

    // disable pin control ??
    val = 0xFF;
    si5351_write(SI5351_REG9_OEB_PIN_ENABLE_CONTROL, val, 1);
}