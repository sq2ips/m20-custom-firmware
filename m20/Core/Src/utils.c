#include "utils.h"

float Log(float x) {
	// Trap illegal values
	if (x <= 0) {
		return 0.0;
	}

	// Confine x to a sensible range
	int power_adjust = 0;
	while (x > 1.0) {
		x /= EULER_CONST;
		power_adjust++;
	}
	while (x < .25) {
		x *= EULER_CONST;
		power_adjust--;
	}

	// Now use the Taylor series to calculate the logarithm
	x -= 1.0;
	float t = 0.0, s = 1.0, z = x;
	for (int k = 1; k <= TAYLOR_ITERATIONS; k++) {
		t += z * s / k;
		z *= x;
		s = -s;
	}

	// Combine the result with the power_adjust value and return
	return t + power_adjust;
}

int32_t Round(float number) {
	// Get the integer part by truncating the float
	int int_part = (int)number;
	float fractional_part = number - int_part;
	if (fractional_part >= 0.5) {
		return int_part + 1;
	} else {
		return int_part;
	}
}

uint32_t convert_buffer_to_uint32(const uint8_t* buffer, const uint8_t size) {
	uint32_t result = 0;
	for (int i = 0; i < size; i++) {
		result <<= 8;
		result |= buffer[i];
	}
	return result;
}

// time difference in seconds, handles midnight crossing
int16_t timeDifference(const uint32_t previousTime, const uint32_t currentTime) {
	if (previousTime < currentTime) return (int16_t)(currentTime - previousTime);

	return (int16_t)(24 * 60 * 60 - (previousTime - currentTime));
}

// ascent rate in cm/s
int16_t calculateAscentRate(const uint16_t previousAlt, const uint16_t currentAlt, const uint32_t previousTime, const uint32_t currentTime) {
	const int16_t altDiff = (int16_t)(currentAlt - previousAlt);
	const int16_t timeDiff = timeDifference(previousTime, currentTime);

	return (int16_t)Round((float)altDiff / timeDiff);
}

long long Floor(double num) { // Used only for positive values
    if (num >= 0) {
        return (long long)num; // Truncates positive numbers down
    } else {
        // Handle negative numbers separately if needed, or rely on the logic below
        return (long long)num; // For negative, this truncates towards zero (e.g., -4.7 becomes -4)
    }
}

uint16_t crc16(char* string, uint8_t len) {
	uint16_t crc = 0xffff;
	char i;
	uint8_t ptr = 0;
	while (ptr < len) {
		ptr++;
		crc = crc ^ (*(string++) << 8);
		for (i = 0; i < 8; i++) {
			if (crc & 0x8000)
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			else
				crc <<= 1;
		}
	}
	return crc;
}