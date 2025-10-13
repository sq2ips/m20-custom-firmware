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
    for (int k=1; k<=TAYLOR_ITERATIONS; k++) {
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
static float normalize_angle(float x) {
    while (x > PI) x -= 2 * PI;
    while (x < -PI) x += 2 * PI;
    return x;
}

float Sinf(float x) {
    x = normalize_angle(x);

    // Taylor series expansion up to x^9 term
    float x2 = x * x;
    float term = x;
    float result = term;

    term *= -x2 / (2 * 3);   // -x^3 / 3!
    result += term;

    term *= -x2 / (4 * 5);   // +x^5 / 5!
    result += term;

    term *= -x2 / (6 * 7);   // -x^7 / 7!
    result += term;

    term *= -x2 / (8 * 9);   // +x^9 / 9!
    result += term;

    return result;
}