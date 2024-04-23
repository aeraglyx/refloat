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

    float yaw_diff;
    float yaw_diff_clean;
    float yaw_last;

    float pitch_balance;

    float gyro[3];
} IMUData;

void imu_data_reset(IMUData *imu);

// void motor_data_configure(MotorData *m, float frequency);

void imu_data_update(IMUData *imu, BalanceFilterData *balance_filter);
