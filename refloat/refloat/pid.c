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

void pid_reset(PID *pid, const CfgPid *cfg, float cooldown_alpha) {
    pid->pid_value = 0.0f;

    pid->proportional = 0.0f;
    // pid->integral = 0.0f;
    filter_ema(&pid->integral, 0.0f, cooldown_alpha);
    pid->derivative = 0.0f;

    pid->kd_filtered = 0.0f;

    // pid->kp_scale = cfg->kp;
    filter_ema(&pid->kp_scale, cfg->kp, cooldown_alpha);
    // pid->kd_scale = cfg->kd;
    filter_ema(&pid->kd_scale, cfg->kd, cooldown_alpha);

    pid->soft_start_factor = 0.0f;
}

void pid_configure(PID *pid, const CfgPid *cfg, float dt) {
    pid->kd_alpha = half_time_to_alpha(cfg->kd_filter, dt);
    pid->z_alpha = half_time_to_alpha(cfg->drop_filter, dt);
    pid->ki = cfg->ki * dt;
    pid->soft_start_step_size = dt / max(cfg->soft_start, dt);
}

static void p_update(PID *pid, const CfgPid *cfg, float pitch_offset, int8_t direction, float brake_factor) {
    float kp = cfg->kp;
    if (sign(pitch_offset) != direction) {
        // TODO only when kp_brake_scale != 0 ?
        const float kp_brake_scale = 1.0f + (cfg->kp_brake - 1.0f) * brake_factor;
        kp *= kp_brake_scale;
    }
    filter_ema(&pid->kp_scale, kp, 0.01f);
    pid->proportional = pitch_offset * pid->kp_scale;
}

static void i_update(PID *pid, const CfgPid *cfg, float pitch_offset) {
    pid->integral += pitch_offset * pid->ki;
    if (cfg->ki_limit > 0.0f && fabsf(pid->integral) > cfg->ki_limit) {
        pid->integral = cfg->ki_limit * sign(pid->integral);
    }
    // Quickly ramp down integral component during reverse stop
    // if (d->state.sat == SAT_REVERSESTOP) {
    //     pid->integral = pid->integral * 0.9;
    // }
}

static void d_update(PID *pid, const CfgPid *cfg, float gyro_y, int8_t direction, float brake_factor) {
    filter_ema(&pid->kd_filtered, -gyro_y, pid->kd_alpha); 
    const float kd_input = pid->kd_filtered;
    // TODO rate p limiting?
    float kd = cfg->kd;
    if (sign(kd_input) != direction) {
        const float kd_brake_scale = 1.0f + (cfg->kd_brake - 1.0f) * brake_factor;
        kd *= kd_brake_scale;
    }
    filter_ema(&pid->kd_scale, kd, 0.01f);
    pid->derivative = kd_input * pid->kd_scale;
}

static void drop_update(PID *pid, const CfgPid *cfg, float accel_z) {
    filter_ema(&pid->z_filtered, accel_z, pid->z_alpha);
    const float factor = clamp(1.0f - pid->z_filtered / cfg->drop_spread, 0.0f, 1.0f);
    pid->drop_mult = 1.0f - (1.0f - cfg->drop_strength) * factor;
}

// static void f_update(PID *pid, const CfgPid *cfg, float gyro_y, int8_t direction, float brake_factor) {
//     const float speed = 
//     pid->feed_forward = speed * pid->kf_scale;
// }

void pid_update(
    PID *pid, const IMUData *imu, const MotorData *mot, const CfgPid *cfg, float setpoint
) {
    const float brake_factor = clamp(fabsf(mot->speed_smooth), 0.0f, 1.0f);

    const float pitch_offset = setpoint - imu->pitch_balance;
    const int8_t direction = sign(mot->speed_smooth);

    p_update(pid, cfg, pitch_offset, direction, brake_factor);
    i_update(pid, cfg, pitch_offset);
    d_update(pid, cfg, imu->gyro[1], direction, brake_factor);
    
    drop_update(pid, cfg, imu->accel_derotated[2]);

    // FEED FORWARD
    // TODO

    float new_pid_value = pid->proportional + pid->integral + pid->derivative;
    new_pid_value *= pid->drop_mult;
    // TODO speed boost

    // CURRENT LIMITING
    float current_limit = mot->braking ? mot->current_min : mot->current_max;
    new_pid_value = clamp_sym(new_pid_value, current_limit);
    // TODO soft max?

    // SOFT START
    // after limiting, otherwise soft start wouldn't be effective with aggressive PIDs
    if (pid->soft_start_factor < 1.0f) {
        const float factor_new = pid->soft_start_factor + pid->soft_start_step_size;
        pid->soft_start_factor = clamp(factor_new, 0.0f, 1.0f);
        new_pid_value *= pid->soft_start_factor;
    }

    filter_ema(&pid->pid_value, new_pid_value, 0.2f);
}
