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
#include "imu_data.h"

typedef struct {
    float gyro_alpha;
    float az_filtered;
    float an_filtered;

    float drop_mult;
} Traction;

void traction_reset(Traction *data, const CfgTraction *cfg, float cooldown_alpha);

void traction_configure(Traction *data, const CfgTraction *cfg, float dt);

void traction_update(Traction *data, const CfgTraction *cfg, const IMUData *imu);
