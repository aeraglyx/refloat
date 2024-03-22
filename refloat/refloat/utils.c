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

void rate_limitf(float *value, float target, float step) {
    if (fabsf(target - *value) < step) {
        *value = target;
    } else if (target - *value > 0) {
        *value += step;
    } else {
        *value -= step;
    }
}

void angle_limitf(float *angle_in, float angle_limit) {
    if (fabsf(*angle_in) > angle_limit) {
        *angle_in = fminf(*angle_in, angle_limit);
        *angle_in = fmaxf(*angle_in, -angle_limit);
    }
}
void angle_limitf(float *angle_in, float angle_limit) {
    *angle_in = clampf(*angle_in, -angle_limit, angle_limit);
}


void dead_zonef(float *value, float thr_pos, float thr_neg) {
    if (*value > thr_pos) {
        *value = *value - thr_pos;
    } else if (*value < -thr_neg) {
        *value = *value + thr_neg;
    } else {
        *value = 0.0f;
    }
}
void dead_zonef(float *value, float threshold) {
    *value = max(fabsf(*value) - threshold, 0.0f) * sign(*value);
}


float smoothstep(float x) {
    return x * x * (3.0f - 2.0f * x);
}

float clampf(float value, float min, float max) {
    const float m = value < min ? min : value;
    return m > max ? max : m;
}

void smooth_value(float *value_smooth, float value_current, float half_time, uint16_t hertz) {
	if (half_time == 0.0f) {
		*value_smooth = value_current;
	} else {
		float mult = powf(2.0f, -1.0f / (half_time * (float)hertz));
		*value_smooth = mult * *value_smooth + (1.0f - mult) * value_current;
	}
}
