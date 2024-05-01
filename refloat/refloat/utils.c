
// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "utils.h"

#include <math.h>

uint32_t rnd(uint32_t seed) {
    return seed * 1664525u + 1013904223u;
}


float clamp(float value, float min, float max) {
    const float m = value < min ? min : value;
    return m > max ? max : m;
}

void clamp_sym(float *angle_in, float angle_limit) {
    *angle_in = clamp(*angle_in, -angle_limit, angle_limit);
}

void dead_zonef(float *value, float threshold) {
    *value = fmaxf(fabsf(*value) - threshold, 0.0f) * sign(*value);
}

void rate_limitf(float *value, float target, float step) {
    if (fabsf(target - *value) < step) {
        *value = target;
    } else if (target - *value > 0) {
        *value += step;
    } else {
        *value -= step;
    }
}

// float rate_limit_v04(float interpolated, float target, float step, float ramp) {
//     float offset = target - interpolated;
//     float step_multiplier = clamp(offset / fmaxf(ramp, 0.01f), -1.0f, 1.0f);
//     return step * step_multiplier;
// }

float tilt_speed(float interpolated, float target, float speed, float speed_max) {
    float offset = target - interpolated;
    return clamp(offset * speed, -speed_max, speed_max);
}

// void dead_zonef(float *value, float thr_pos, float thr_neg) {
//     if (*value > thr_pos) {
//         *value = *value - thr_pos;
//     } else if (*value < -thr_neg) {
//         *value = *value + thr_neg;
//     } else {
//         *value = 0.0f;
//     }
// }

void smooth_value(float *value_smooth, float value_current, float half_time_sec, uint16_t hertz) {
    if (half_time_sec < 0.001f) {
        *value_smooth = value_current;
    } else {
        float mult = powf(0.5f, 1.0f / (half_time_sec * (float)hertz));
        *value_smooth = mult * (*value_smooth) + (1.0f - mult) * value_current;
    }
}

float smoothing_factor(float half_time_sec, uint16_t hertz) {
    if (half_time_sec < 0.001f) {
        return 1.0f;
    }
    return powf(0.5f, 1.0f / (half_time_sec * (float)hertz));
}

// float smoothstep(float x) {
//     return x * x * (3.0f - 2.0f * x);
// }

// float sigmoid(float x, float radius) {
//     return tanhf(x / radius);
// }

// float sigmoid_norm(float x, float radius) {
//     return 0.5f + 0.5f * sigmoid(x, radius);
// }

// float remap(float x, float a, float b) {
//     // remap -1 to a and 1 to b
//     return 0.5f * (a - a * x + b + b * x);
// }

// float remap_norm(float x, float a, float b) {
//     return (1.0f - x) * a + x * b;
// }
