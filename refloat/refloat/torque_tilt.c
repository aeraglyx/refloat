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
    tt->debug = 0.0f;
    tt->accel_offset_smooth = 0.0f;
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *cfg) {
    // tt->step_size_on = cfg->torquetilt_speed_max_on / cfg->hertz;
    // tt->step_size_off = cfg->torquetilt_speed_max_off / cfg->hertz;
}

void torque_tilt_update(TorqueTilt *tt, const MotorData *mot, const RefloatConfig *cfg, const IMUData *imu) {
    float current = mot->current_filtered;
    float accel_factor = cfg->atr_amps_accel_ratio;
    float current_based_on_accel = mot->accel_clamped * accel_factor;

    float alpha = half_time_to_alpha(cfg->torquetilt_filter, cfg->hertz);
    float accel_offset = current_based_on_accel - current;
    filter_ema(&tt->accel_offset_smooth, accel_offset, alpha);
    float target = current + tt->accel_offset_smooth * cfg->torquetilt_method;

    dead_zonef(&target, cfg->torquetilt_start_current);
    float strength =
        mot->braking ? cfg->torquetilt_strength_regen : cfg->torquetilt_strength;

    float turn_boost = 1.0f + fabsf(imu->yaw_rate) * cfg->torquetilt_turn_boost * 0.00125f;
    strength *= turn_boost;

    target *= strength;
    target = clamp_sym(target, cfg->torquetilt_angle_limit);

    const float offset = target - tt->interpolated;
    float speed = offset * cfg->torquetilt_speed;
    speed = clamp_sym(speed, cfg->torquetilt_speed_max_on);

    tt->interpolated += speed / cfg->hertz;

    tt->debug = imu->yaw_rate;
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->interpolated *= 0.995;
}
