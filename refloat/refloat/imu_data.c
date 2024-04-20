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

void imu_data_reset(IMUData *imu) {
    imu->yaw_last = 0.0f;
    imu->yaw_diff = 0.0f;
}

// void imu_data_configure(IMUData *imu, float frequency) {
//     if (frequency > 0) {
//         biquad_configure(&imu->atr_current_biquad, BQ_LOWPASS, frequency);
//         imu->atr_filter_enabled = true;
//     } else {
//         imu->atr_filter_enabled = false;
//     }
// }

void imu_data_update(IMUData *imu, BalanceFilterData *balance_filter) {
    // TODO rad2deg(VESC_IF->ahrs_get_pitch(&d->m_att_ref))
    imu->pitch = rad2deg(VESC_IF->imu_get_pitch());
    imu->roll = rad2deg(VESC_IF->imu_get_roll());
    imu->yaw = rad2deg(VESC_IF->imu_get_yaw());

    // imu->pitch_diff = imu->pitch - imu->pitch_last;
    // imu->roll_diff = imu->roll - imu->roll_last;
    float yaw_diff_raw = imu->yaw - imu->yaw_last;
    if ((yaw_diff_raw == 0) || (fabsf(yaw_diff_raw) > 90)) {
        yaw_diff_raw = 0.0f;
    }
    imu->yaw_diff = 0.8f * imu->yaw_diff + 0.2f * yaw_diff_raw;

    // imu->pitch_last = imu->pitch;
    // imu->roll_last = imu->roll;
    imu->yaw_last = imu->yaw;

    imu->pitch_balance = rad2deg(balance_filter_get_pitch(balance_filter));
    // imu->balance_roll = rad2deg(balance_filter_get_roll(&balance_filter));
    // imu->balance_yaw = rad2deg(balance_filter_get_yaw(&balance_filter));
    
    VESC_IF->imu_get_gyro(imu->gyro);
}
