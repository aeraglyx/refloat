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
    // st->step_size = cfg->speedtilt_speed_max / cfg->hertz;
    st->linear_converted = cfg->speedtilt_variable / 1000;
}

void speed_tilt_update(SpeedTilt *st, const MotorData *mot, const RefloatConfig *cfg) {
    float linear = mot->erpm_smooth * st->linear_converted;
    angle_limitf(&linear, cfg->speedtilt_variable_max);
    float constant = clampf(mot->erpm_smooth / 500, -1.0f, 1.0f) * cfg->speedtilt_constant;
    float target = linear + constant;

    float speed = tilt_speed(st->interpolated, target, cfg->speedtilt_speed, cfg->speedtilt_speed_max);

    float interpolated_new = st->interpolated + speed / cfg->hertz;
    smooth_value(&st->interpolated, interpolated_new, cfg->tiltback_filter, cfg->hertz);
}

void speed_tilt_winddown(SpeedTilt *st) {
    st->interpolated *= 0.998;
}
