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

#include "turn_tilt.h"
#include "utils.h"

#include <math.h>

void turn_tilt_reset(TurnTilt *tt, float cooldown_alpha) {
    // tt->target = 0.0f;
    // tt->interpolated = 0.0f;
    filter_ema(&tt->interpolated, 0.0f, cooldown_alpha);
}

// void turn_tilt_configure(TurnTilt *tt, const RefloatConfig *cfg) {
// }

void turn_tilt_update(TurnTilt *tt, const MotorData *mot, const IMUData *imu, const ATR *atr, const CfgTurnTilt *cfg, float dt) {
    if (cfg->strength == 0) {
        return;
    }

    tt->target = fabsf(imu->yaw_rate) * cfg->strength * 0.00125;

    float speed_boost = powf(cfg->strength_boost, mot->erpm_abs_10k);
    tt->target *= speed_boost;

    float start_erpm = max(cfg->start_erpm, 10);
    float direction = clamp_sym(mot->erpm_smooth / start_erpm, 1.0f);
    tt->target *= direction;

    // ATR interference: Reduce target during moments of high torque response
    float atr_min = 2;
    float atr_max = 5;
    if (fabsf(atr->target) > atr_min){
        // Start scaling turntilt when ATR>2, down to 0 turntilt for ATR > 5 degrees
        float atr_scaling = (atr_max - fabsf(atr->target)) / (atr_max - atr_min);
        if (atr_scaling < 0){
            atr_scaling = 0;
        }
        tt->target *= atr_scaling;
    }
    // if (fabsf(d->imu.pitch_balance - d->noseangling_interpolated) > 4) {
    //     // no setpoint changes during heavy acceleration or braking
    //     tt->target = 0;
    // }

    tt->target = clamp_sym(tt->target, cfg->angle_limit);

    const float offset = tt->target - tt->interpolated;
    float speed = offset * cfg->speed;
    // clamp_sym(&speed, cfg->turntilt_speed_max);
    speed = clamp_sym(speed, cfg->speed_max);

    tt->interpolated += speed * dt;
}

void turn_tilt_winddown(TurnTilt *tt) {
    tt->interpolated *= 0.995;
}
