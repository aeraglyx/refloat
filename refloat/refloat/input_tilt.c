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

#include "input_tilt.h"
#include "utils.h"

#include <math.h>

void input_tilt_reset(InputTilt *it) {
    it->interpolated = 0.0f;
}

void input_tilt_configure(InputTilt *it, const RefloatConfig *cfg) {
    // it->inputtilt_step_size = cfg->inputtilt_speed_max / cfg->hertz;
}

void input_tilt_update(InputTilt *it, const RemoteData *remote, const RefloatConfig *cfg) {
    float target = remote->throttle_filtered * cfg->inputtilt_angle_limit;

    const float offset = target - it->interpolated;
    float speed = offset * cfg->inputtilt_speed;
    speed = clamp_sym(speed, cfg->inputtilt_speed_max);

    it->interpolated += speed / cfg->hertz;
}
