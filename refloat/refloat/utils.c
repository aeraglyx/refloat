
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

float clamp_sym(float angle_in, float angle_limit) {
    return clamp(angle_in, -angle_limit, angle_limit);
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

// void dead_zonef(float *value, float thr_pos, float thr_neg) {
//     if (*value > thr_pos) {
//         *value = *value - thr_pos;
//     } else if (*value < -thr_neg) {
//         *value = *value + thr_neg;
//     } else {
//         *value = 0.0f;
//     }
// }

float half_time_to_alpha(float half_time, float dt) {
    if (half_time < 0.005f) {
        return 1.0f;
    }
    return 1.0f - exp2f(-dt / half_time);  // true value
}

float half_time_to_alpha_fast(float half_time, float dt) {
    if (half_time < 0.005f) {
        return 1.0f;
    }
    return 0.69f * dt / half_time;  // approximation
}

void filter_ema(float *value_smooth, float value_new, float alpha) {
    *value_smooth = *value_smooth * (1.0f - alpha) + value_new * alpha;
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
