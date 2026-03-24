#pragma once
#include <stdint.h>

// 2^x approximation, x in [-10, 20], max relative error < 0.2%
inline float fast_exp2(float x)
{
    int n = (int)x;
    float f = x - (float)n;
    if (f < 0.0f) { n--; f += 1.0f; } // floor, not truncate

    // 4-term Taylor: coefficients are (ln2)^k / k!
    float p = 1.0f + f * (0.6931472f
            + f * (0.2402265f
            + f * (0.0555041f
            + f *  0.0096174f)));

    // apply 2^n exactly via IEEE754 exponent field
    union { float f; int32_t i; } u;
    u.i = (n + 127) << 23;
    return p * u.f;
}

// cos(t * pi/2) for t in [0, 1], max absolute error < 0.002
inline float fast_cos_pan(float t)
{
    float t2 = t * t;
    return 1.0f + t2 * (-1.2337006f
           + t2 * ( 0.2536763f
           + t2 * (-0.0208606f)));
}
