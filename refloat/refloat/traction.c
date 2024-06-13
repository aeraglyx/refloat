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

#include "traction.h"
#include "utils.h"

#include <math.h>
#include <stdint.h>

void traction_reset(Traction *data, const CfgTraction *cfg, float cooldown_alpha) {
    // data->pid_value = 0.0f;
}

void traction_configure(Traction *data, const CfgTraction *cfg, float dt) {
    data->gyro_alpha = half_time_to_alpha(cfg->drop_filter, dt);
}

void traction_update(Traction *data, const CfgTraction *cfg, const IMUData *imu) {
    const float ax = imu->accel_derotated[0];
    const float ay = imu->accel_derotated[1];
    const float az = imu->accel_derotated[2];

    filter_ema(&data->az_filtered, az, data->gyro_alpha);
    const float an = sqrtf(ax * ax + ay * ay);
    filter_ema(&data->an_filtered, an, data->gyro_alpha);

    // const float factor_z = clamp(1.0f - data->az_filtered / cfg->drop_threshold_z, 0.0f, 1.0f);
    // const float factor_n = clamp(1.0f - data->an_filtered / cfg->drop_threshold_n, 0.0f, 1.0f);
    const bool factor_z = data->az_filtered < cfg->drop_threshold_z;
    const bool factor_n = fabsf(data->an_filtered) < cfg->drop_threshold_n;
    const bool factor = factor_z && factor_n;
    data->drop_mult = 1.0f - (1.0f - cfg->drop_strength) * (float)factor;
}
