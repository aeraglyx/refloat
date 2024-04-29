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

#pragma once

#include "conf/datatypes.h"
#include "motor_data.h"

typedef struct {
    float step_size;
    float interpolated;
    // float step_smooth;
    float tiltback_variable;
    float tiltback_variable_max_erpm;
} SpeedTilt;

void speed_tilt_reset(SpeedTilt *tt);

void speed_tilt_configure(SpeedTilt *tt, const RefloatConfig *config);

void speed_tilt_update(SpeedTilt *tt, const MotorData *motor, const RefloatConfig *config);

void speed_tilt_winddown(SpeedTilt *tt);
