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
    tt->offset = 0.0f;
    tt->step_smooth = 0.0f;
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *cfg) {
    tt->on_step_size = cfg->torquetilt_on_speed / cfg->hertz;
    tt->off_step_size = cfg->torquetilt_off_speed / cfg->hertz;
}

void torque_tilt_update(TorqueTilt *tt, const MotorData *mot, const RefloatConfig *cfg) {
    float strength =
        mot->braking ? cfg->torquetilt_strength_regen : cfg->torquetilt_strength;
    float accel_factor = cfg->atr_amps_accel_ratio;

    // float amp_offset_speed = 0.00022f * mot->erpm_smooth * accel_factor;
    // float amp_offset_atr = accel_diff * accel_factor;
    // float amps_adjusted = mot->atr_filtered_current - amp_offset_speed - amp_offset_atr;

    float current_filtered = mot->current_filtered;
    float current_based_on_accel = mot->accel_clamped * accel_factor;
    // float acceleration = clampf(mot->acceleration, -5.0f, 5.0f);
    // float current_based_on_accel = acceleration * accel_factor;

    float method = cfg->brkbooster_angle;
    float target_offset = (1.0f - method) * current_filtered + method * current_based_on_accel;

    dead_zonef(&target_offset, cfg->torquetilt_start_current);
    target_offset *= strength;
    angle_limitf(&target_offset, cfg->torquetilt_angle_limit);
    
    float ramp = cfg->booster_angle;
    float offset = target_offset - tt->offset;
    float step_max = tt->on_step_size;
    float step = get_step(offset, step_max, ramp);
    smooth_value(&tt->step_smooth, step, ramp * 0.05f, cfg->hertz);
    tt->offset += tt->step_smooth;
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->offset *= 0.995;
}
