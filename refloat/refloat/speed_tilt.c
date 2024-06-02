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

void speed_tilt_reset(SpeedTilt *st, float cooldown_alpha) {
    filter_ema(&st->interpolated, 0.0f, cooldown_alpha);
}

void speed_tilt_configure(SpeedTilt *st, const CfgSpeedTilt *cfg) {
    st->linear_converted = cfg->variable * 0.001f;
}

void speed_tilt_update(SpeedTilt *st, const MotorData *mot, const CfgSpeedTilt *cfg, float dt) {
    float variable = mot->speed_smooth * cfg->variable;
    variable = clamp_sym(variable, cfg->variable_max);

    const float constant_lerp = clamp_sym(mot->speed_smooth * 2.0f, 1.0f);
    const float constant = constant_lerp * cfg->constant;

    const float target = variable + constant;

    const float offset = target - st->interpolated;
    const float speed = clamp_sym(offset * cfg->speed, cfg->speed_max);

    st->interpolated += speed * dt;
}

void speed_tilt_winddown(SpeedTilt *st) {
    st->interpolated *= 0.998;
}
