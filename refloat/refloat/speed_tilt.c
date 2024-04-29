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

#include "speed_tilt.h"
#include "utils.h"

#include <math.h>

void speed_tilt_reset(SpeedTilt *st) {
    st->interpolated = 0.0f;
    // st->step_smooth = 0.0f;
}

void speed_tilt_configure(SpeedTilt *st, const RefloatConfig *cfg) {
    st->step_size = cfg->noseangling_speed / cfg->hertz;

    // Variable nose angle adjustment / tiltback (setting is per 1000erpm, convert to per erpm)
    st->tiltback_variable = cfg->tiltback_variable / 1000;
    if (st->tiltback_variable > 0) {
        st->tiltback_variable_max_erpm =
            fabsf(cfg->tiltback_variable_max / st->tiltback_variable);
    } else {
        st->tiltback_variable_max_erpm = 100000;
    }
}

void speed_tilt_update(SpeedTilt *st, const MotorData *mot, const RefloatConfig *cfg) {
    // Variable Tiltback looks at ERPM from the reference point of the set minimum ERPM
    // float variable_erpm = fmaxf(0, d->motor.erpm_abs - d->float_conf.tiltback_variable_erpm);
    float target = mot->erpm_smooth * st->tiltback_variable;
    angle_limitf(&target, cfg->tiltback_variable_max);

    // if (variable_erpm > d->tiltback_variable_max_erpm) {
    //     target = d->float_conf.tiltback_variable_max * d->motor.erpm_sign;
    // } else {
    //     target = d->tiltback_variable * variable_erpm * d->motor.erpm_sign *
    //         sign(d->float_conf.tiltback_variable_max);
    // }

    // TODO constant
    // if (d->motor.erpm_abs > d->float_conf.tiltback_constant_erpm) {
    //     target += d->float_conf.tiltback_constant * d->motor.erpm_sign;
    // }

    // float ramp = 0.1f;  // TODO
    // float half_time = ramp * 0.5f;

    float speed = tilt_speed(st->interpolated, target, cfg->turntilt_speed, cfg->noseangling_speed);

    float interpolated_new = st->interpolated + speed / cfg->hertz;
    smooth_value(&st->interpolated, interpolated_new, cfg->tiltback_filter, cfg->hertz);

    // float step_new = rate_limit_v04(st->interpolated, target, st->step_size, ramp);
    // smooth_value(&st->step_smooth, step_new, half_time, cfg->hertz);
    // st->interpolated += st->step_smooth;
}

void speed_tilt_winddown(SpeedTilt *st) {
    st->interpolated *= 0.998;
}
