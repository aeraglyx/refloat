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
    tt->step_smooth = 0.0f;
}

void turn_tilt_configure(TurnTilt *tt, const RefloatConfig *cfg) {
    tt->step_size = cfg->turntilt_speed / cfg->hertz;
}

void turn_tilt_update(TurnTilt *tt, const MotorData *mot, const IMUData *imu, const ATR *atr, const RefloatConfig *cfg) {
    if (cfg->turntilt_strength == 0) {
        return;
    }

    tt->target = fabsf(imu->yaw_diff) * cfg->turntilt_strength;

    float speed_boost = powf(cfg->turntilt_erpm_boost, fabsf(mot->erpm_smooth) * 0.0001f);
    tt->target *= speed_boost;

    // Disable below erpm threshold otherwise add directionality
    if (mot->abs_erpm < cfg->turntilt_start_erpm) {
        tt->target = 0.0f;
    } else {
        tt->target *= mot->erpm_sign;
    }

    // ATR interference: Reduce target during moments of high torque response
    float atr_min = 2;
    float atr_max = 5;
    if (fabsf(atr->target) > atr_min){
        // Start scaling turntilt when ATR>2, down to 0 turntilt for ATR > 5 degrees
        float atr_scaling = (atr_max - fabsf(atr->target)) / (atr_max - atr_min);
        if (atr_scaling < 0){
            atr_scaling = 0;
            // during heavy torque response clear the yaw aggregate too
        }
        tt->target *= atr_scaling;
    }
    // if (fabsf(d->imu.pitch_balance - d->noseangling_interpolated) > 4) {
    //     // no setpoint changes during heavy acceleration or braking
    //     tt->target = 0;
    // }

    angle_limitf(&tt->target, cfg->turntilt_angle_limit);
    rate_limitf(&tt->interpolated, tt->target, tt->step_size);

    float ramp = cfg->turntilt_ramp;
    float half_time = ramp * 0.5f;

    float step_new = rate_limit_v04(tt->interpolated, tt->target, tt->step_size, ramp);
    smooth_value(&tt->step_smooth, step_new, half_time, cfg->hertz);
    tt->interpolated += tt->step_smooth;
}

// TODO
// void turn_tilt_winddown(TurnTilt *tt) {
//     tt->interpolated *= 0.995;
// }
