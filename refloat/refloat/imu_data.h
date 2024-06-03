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

#include "balance_filter.h"
#include "conf/datatypes.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pitch;
    float roll;
    float yaw;

    float pitch_balance;

    float gyro[3];

    float yaw_rate;
    float yaw_rate_alpha;

    float accel[3];
    float accel_derotated[3];

    float gy_last;
    float az_corrected;
    float angular_to_linear_acc;
} IMUData;

void imu_data_init(IMUData *imu);

// void imu_data_reset(IMUData *imu, float cooldown_alpha);

void imu_data_configure(IMUData *imu, const CfgTurnTilt *cfg, const CfgHwEsc *esc, float dt);

void imu_data_update(IMUData *imu, BalanceFilterData *balance_filter);
