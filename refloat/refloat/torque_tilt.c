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

void torque_tilt_reset(TorqueTilt *tt, float cooldown_alpha) {
    // tt->interpolated = 0.0f;
    filter_ema(&tt->interpolated, 0.0f, cooldown_alpha);
    // tt->accel_offset_smooth = 0.0f;
    filter_ema(&tt->accel_offset_smooth, 0.0f, cooldown_alpha);
    // tt->debug = 0.0f;
}

// void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *cfg) {
// }

void torque_tilt_update(
    TorqueTilt *tt,
    const MotorData *mot,
    const IMUData *imu,
    const CfgTorqueTilt *cfg,
    const CfgAtr *cfg_atr,
    float dt
) {
    const float current = mot->current_filtered;
    const float accel_factor = cfg_atr->amps_accel_ratio;
    const float current_based_on_accel = mot->accel_clamped * accel_factor;

    const float alpha = half_time_to_alpha(cfg->filter, dt);
    const float accel_offset = current_based_on_accel - current;
    filter_ema(&tt->accel_offset_smooth, accel_offset, alpha);
    float target = current + tt->accel_offset_smooth * cfg->method;

    dead_zonef(&target, cfg->start_current);
    float strength = mot->braking ? cfg->strength_regen : cfg->strength;
    
    const float strength_boost = powf(cfg->strength_boost, mot->erpm_abs_10k);
    strength *= strength_boost;

    const float turn_boost = 1.0f + fabsf(imu->yaw_rate) * cfg->turn_boost * 0.00125f;
    strength *= turn_boost;

    target *= strength;
    target = clamp_sym(target, cfg->angle_limit);

    const float offset = target - tt->interpolated;
    float speed = offset * cfg->speed;
    speed = clamp_sym(speed, cfg->speed_max);

    tt->interpolated += speed * dt;

    // tt->debug = imu->yaw_rate;
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->interpolated *= 0.995;
}
