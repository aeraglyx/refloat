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

void turn_tilt_reset(TurnTilt *tt) {
    tt->target = 0.0f;
    tt->interpolated = 0.0f;
}

void turn_tilt_configure(TurnTilt *tt, const RefloatConfig *config) {
    tt->step_size = config->turntilt_speed / config->hertz;
}

void turn_tilt_update(TurnTilt *tt, const MotorData *motor, IMUData *imu, ATR *atr, const RefloatConfig *config) {
    if (config->turntilt_strength == 0) {
        return;
    }

    tt->target = fabsf(imu->yaw_diff) * config->turntilt_strength;

    // Apply speed scaling
    // float boost;
    // if (motor->abs_erpm < config->turntilt_erpm_boost_end) {
    //     boost = 1.0f + motor->abs_erpm * d->turntilt_boost_per_erpm;
    // } else {
    //     boost = 1.0f + (float) config->turntilt_erpm_boost / 100.0f;
    // }
    // tt->target *= boost;

    // Limit angle to max angle
    if (tt->target > 0.0f) {
        tt->target = fminf(tt->target, config->turntilt_angle_limit);
    } else {
        tt->target = fmaxf(tt->target, -config->turntilt_angle_limit);
    }

    // Disable below erpm threshold otherwise add directionality
    if (motor->abs_erpm < config->turntilt_start_erpm) {
        tt->target = 0.0f;
    } else {
        tt->target *= motor->erpm_sign;
    }

    // ATR interference: Reduce target during moments of high torque response
    float atr_min = 2;
    float atr_max = 5;
    // if (sign(atr->target_offset) != sign(tt->target)) {
    //     // further reduced turntilt during moderate to steep downhills
    //     atr_min = 1;
    //     atr_max = 4;
    // }
    if (fabsf(atr->target_offset) > atr_min){
        // Start scaling turntilt when ATR>2, down to 0 turntilt for ATR > 5 degrees
        float atr_scaling = (atr_max - fabsf(atr->target_offset)) / (atr_max - atr_min);
        if (atr_scaling < 0)
        {
            atr_scaling = 0;
            // during heavy torque response clear the yaw aggregate too
            // d->yaw_aggregate = 0;
        }
        tt->target *= atr_scaling;
    }
    // if (fabsf(d->balance_pitch - d->noseangling_interpolated) > 4) {
    //     // no setpoint changes during heavy acceleration or braking
    //     tt->target = 0;
    //     d->yaw_aggregate = 0;
    // }

    // Move towards target limited by max speed
    rate_limitf(&tt->interpolated, tt->target, tt->step_size);
    // d->setpoint += tt->interpolated;
}

// void turn_tilt_winddown(TurnTilt *tt) {
//     tt->offset *= 0.995;
// }
