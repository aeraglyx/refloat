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
    // not used, update is outside RUNNING
    // filter_ema(&data->az, 1.0f, cooldown_alpha);
    // filter_ema(&data->an, 0.0f, cooldown_alpha);
}

void traction_configure(Traction *data, const CfgTraction *cfg, float dt) {
    data->_gyro_alpha = half_time_to_alpha(cfg->drop_filter, dt);
}

void drop_update(Traction *data, const CfgTraction *cfg, const IMUData *imu) {
    const float ax = imu->accel_derotated[0];
    const float ay = imu->accel_derotated[1];
    const float az = imu->accel_derotated[2];

    filter_ema(&data->ax, ax, data->_gyro_alpha);
    filter_ema(&data->ay, ay, data->_gyro_alpha);
    filter_ema(&data->az, az, data->_gyro_alpha);
    data->an = sqrtf(data->ax * data->ax + data->ay * data->ay);

    // const float factor_z = clamp(1.0f - data->az_filtered / cfg->drop_threshold_z, 0.0f, 1.0f);
    // const float factor_n = clamp(1.0f - data->an_filtered / cfg->drop_threshold_n, 0.0f, 1.0f);
    const bool factor_z = data->az < cfg->drop_threshold_z;
    const bool factor_n = fabsf(data->an) < cfg->drop_threshold_n;
    const bool factor = factor_z && factor_n;
    data->drop_factor = (float)factor;
}

void wheelslip_update(Traction *data, const CfgTraction *cfg, const MotorData *mot) {
    const float accel_factor = clamp_01((fabsf(mot->acceleration) - 5.0f) * 0.2f);
    const float speed_factor = mot->speed_abs * 0.5f;
    const float duty_factor = mot->duty_cycle * 2.0f;
    // const float direction_factor = sign(d->motor.acceleration) == d->motor.speed_sign;

    const float factor = accel_factor * speed_factor * duty_factor * !mot->braking;
    data->wheelslip_factor = clamp_01(factor);
}

void traction_update(Traction *data, const CfgTraction *cfg, const IMUData *imu, const MotorData *mot) {
    drop_update(data, cfg, imu);
    wheelslip_update(data, cfg, mot);

    data->drop_mult = 1.0f - (1.0f - cfg->drop_strength) * data->drop_factor;
}
