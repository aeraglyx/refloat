// Copyright 2022 Dado Mista
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

#include "torque_tilt.h"
#include "utils.h"

#include <math.h>

void torque_tilt_reset(TorqueTilt *tt) {
    tt->interpolated = 0.0f;
    tt->step_smooth = 0.0f;
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *cfg) {
    tt->step_size_on = cfg->torquetilt_on_speed / cfg->hertz;
    tt->step_size_off = cfg->torquetilt_off_speed / cfg->hertz;
}

void torque_tilt_update(TorqueTilt *tt, const MotorData *mot, const RefloatConfig *cfg) {
    float strength =
        mot->braking ? cfg->torquetilt_strength_regen : cfg->torquetilt_strength;
    float accel_factor = cfg->atr_amps_accel_ratio;

    float current = mot->current_filtered;
    float current_based_on_accel = mot->accel_clamped * accel_factor;
    // float acceleration = clampf(mot->acceleration, -5.0f, 5.0f);
    // float current_based_on_accel = acceleration * accel_factor;

    float method = cfg->brkbooster_angle;
    float target = (1.0f - method) * current + method * current_based_on_accel;

    dead_zonef(&target, cfg->torquetilt_start_current);
    target *= strength;
    angle_limitf(&target, cfg->torquetilt_angle_limit);

    float ramp = cfg->booster_angle / 10.0f;
    float half_time = cfg->booster_ramp / 50.0f;

    // float step = set_step(tt->interpolated, target, tt->on_step_size, tt->off_step_size, 0.25f * ramp);
    // rate_limit_v02(&tt->interpolated, target, step, ramp);
    // float interpolated = rate_limit_v03(tt->interpolated, target, step, ramp);
    float step = set_step(tt->interpolated, target, tt->step_size_on, tt->step_size_off);
    float step_new = rate_limit_v04(tt->interpolated, target, step, ramp);
    smooth_value(&tt->step_smooth, step_new, half_time, cfg->hertz);
    tt->interpolated += tt->step_smooth;
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->interpolated *= 0.995;
}
