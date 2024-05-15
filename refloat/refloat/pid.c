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

#include "pid.h"
#include "utils.h"

#include <math.h>
#include <stdint.h>

void pid_reset(PID *pid) {
    pid->pid_value = 0.0f;

    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;

    pid->kd_filtered = 0.0f;

    pid->kp_scale = 1.0f;
    pid->kd_scale = 1.0f;

    pid->soft_start_factor = 0.0f;
}

void pid_configure(PID *pid, const CfgPid *cfg, float dt) {
    pid->kd_alpha = half_time_to_alpha(cfg->kd_filter, dt);
    pid->ki = cfg->ki * dt;
    pid->soft_start_step_size = dt / max(cfg->soft_start, dt);
    pid->expo_a = 1.0f - cfg->kp_expo;
    pid->expo_b = cfg->kp_expo / (cfg->kp_expo_pivot * cfg->kp_expo_pivot);
}

void pid_update(
    PID *pid, const IMUData *imu, const MotorData *mot, const CfgPid *cfg, float setpoint
) {
    const float brake_scale_factor = clamp(fabsf(mot->erpm_smooth) * 0.001f, 0.0f, 1.0f);
    const float kp_brake_scale = 1.0f + (cfg->kp_brake - 1.0f) * brake_scale_factor;
    const float kd_brake_scale = 1.0f + (cfg->kd_brake - 1.0f) * brake_scale_factor;

    const float pitch_offset = setpoint - imu->pitch_balance;
    const int8_t direction = sign(mot->erpm_smooth);

    // PROPORTIONAL
    float kp = cfg->kp;
    if (sign(pitch_offset) != direction) {
        // TODO only when kp_brake_scale != 0 ?
        kp *= kp_brake_scale;
    }
    if (cfg->kp_expo > 0.0f) {
        const float pitch_offset_sq = pitch_offset * pitch_offset;
        const float expo = pid->expo_a + pid->expo_b * pitch_offset_sq;
        kp *= expo;
    }
    filter_ema(&pid->kp_scale, kp, 0.01f);
    pid->proportional = pitch_offset * pid->kp_scale;

    // INTEGRAL
    pid->integral += pitch_offset * pid->ki;
    if (cfg->ki_limit > 0.0f && fabsf(pid->integral) > cfg->ki_limit) {
        pid->integral = cfg->ki_limit * sign(pid->integral);
    }
    // TODO
    // Quickly ramp down integral component during reverse stop
    // if (d->state.sat == SAT_REVERSESTOP) {
    //     pid->integral = pid->integral * 0.9;
    // }

    // DERIVATIVE
    filter_ema(&pid->kd_filtered, -imu->gyro[1], pid->kd_alpha); 
    const float kd_input = pid->kd_filtered;
    // const float kd_input = -imu->gyro[1];
    // TODO rate p limiting?

    float kd = cfg->kd;
    if (sign(kd_input) != direction) {
        kd *= kd_brake_scale;
    }
    filter_ema(&pid->kd_scale, kd, 0.01f);
    pid->derivative = kd_input * pid->kd_scale;

    // FEED FORWARD
    // TODO

    // AGGREGATE
    float new_pid_value = pid->proportional + pid->integral + pid->derivative;
    // TODO speed boost

    // SOFT START
    if (pid->soft_start_factor < 1.0f) {
        new_pid_value *= pid->soft_start_factor;
        const float factor_new = pid->soft_start_factor + pid->soft_start_step_size;
        pid->soft_start_factor = clamp(factor_new, 0.0f, 1.0f);
    }

    // CURRENT LIMITING
    float current_limit = mot->braking ? mot->current_min : mot->current_max;
    new_pid_value = clamp_sym(new_pid_value, current_limit);
    // TODO soft max?

    filter_ema(&pid->pid_value, new_pid_value, 0.2f);
}
