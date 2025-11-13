#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>

#define EULER_CONST 2.718281828459045235
#define TAYLOR_ITERATIONS 20

#define PI 3.14159265358979323846f

float Log(float x);
int32_t Round(float number);
uint32_t convert_buffer_to_uint32(const uint8_t *buffer, uint8_t size);
int16_t timeDifference(uint32_t previousTime, uint32_t currentTime);
int16_t calculateAscentRate(uint16_t previousAlt, uint16_t currentAlt, uint32_t previousTime, uint32_t currentTime);

#endif
