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

void input_tilt_reset(InputTilt *it, const RemoteData *remote) {
    it->interpolated = 0.0f;
    it->throttle_filtered = remote->throttle;
}

void input_tilt_configure(InputTilt *it, const RefloatConfig *cfg) {
}

void input_tilt_update(InputTilt *it, const RemoteData *remote, const CfgInputTilt *cfg, float dt) {
    float target = remote->throttle_filtered * cfg->angle_limit;

    const float offset = target - it->interpolated;
    float speed = offset * cfg->speed;
    speed = clamp_sym(speed, cfg->speed_max);

    it->interpolated += speed * dt;
}
