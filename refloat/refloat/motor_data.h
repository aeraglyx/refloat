
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

#pragma once

#include "conf/datatypes.h"
#include "biquad.h"

#include <stdbool.h>
#include <stdint.h>

// #define ACCEL_ARRAY_SIZE 40

typedef struct {
    float erpm;
    float erpm_filtered;
    float erpm_abs;
    float erpm_last;
    int8_t erpm_sign;
    float erpm_smooth;
    float erpm_abs_10k;

    float current;
    float current_filtered;
    bool braking;

    float duty_cycle;
    float duty_smooth;

    float acceleration;
    float accel_clamped;

    // bool atr_filter_enabled;
    // Biquad atr_current_biquad;

    float filter_half_time;
    float erpm_filter_alpha;
    float atr_filter_alpha;

    float current_min;
    float current_max;
} MotorData;

void motor_data_reset(MotorData *m);

void motor_data_configure(MotorData *m, const CfgTune *cfg, float dt);

void motor_data_update(MotorData *m, uint16_t frequency);
