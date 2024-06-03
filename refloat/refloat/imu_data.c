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

#include "imu_data.h"
#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

void imu_data_init(IMUData *imu) {
    imu->yaw_rate = 0.0f;
}

// void imu_data_reset(IMUData *imu, float cooldown_alpha) {
//     // imu->yaw_rate = 0.0f;
//     filter_ema(&imu->yaw_rate, 0.0f, cooldown_alpha);
// }

void imu_data_configure(IMUData *imu, const CfgTurnTilt *cfg, const CfgHwEsc *esc, float dt) {
    imu->yaw_rate_alpha = half_time_to_alpha(cfg->filter, dt);
    imu->angular_to_linear_acc = esc->imu_x_offset / 9.81f;
}

void imu_data_update(IMUData *imu, BalanceFilterData *balance_filter) {
    imu->pitch = rad2deg(VESC_IF->imu_get_pitch());
    imu->roll = rad2deg(VESC_IF->imu_get_roll());
    imu->yaw = rad2deg(VESC_IF->imu_get_yaw());

    imu->pitch_balance = rad2deg(balance_filter_get_pitch(balance_filter));

    VESC_IF->imu_get_gyro(imu->gyro);

    const float yaw_rate_new = clamp_sym(imu->gyro[2], 200.0f);
    filter_ema(&imu->yaw_rate, yaw_rate_new, imu->yaw_rate_alpha);

    VESC_IF->imu_get_accel(imu->accel);
    VESC_IF->imu_get_accel_derotated(imu->accel_derotated);

    const float pitch_angular_acc = deg2rad(imu->gyro[1] - imu->gy_last);
    imu->gy_last = imu->gyro[1];
    const float az_correction = pitch_angular_acc * imu->angular_to_linear_acc;
    imu->az_corrected = imu->accel[2] - az_correction;

}
