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
    pid->rate_p = 0.0f;

    pid->pitch_rate_filtered = 0.0f;

    pid->kp_brake_scale = 1.0f;
    pid->kd_brake_scale = 1.0f;
    pid->kp_accel_scale = 1.0f;
    pid->kd_accel_scale = 1.0f;

    pid->softstart_pid_limit = 0.0f;
}

void pid_configure(PID *pid, const CfgPid *cfg, float dt) {
    pid->pitch_rate_alpha = half_time_to_alpha(cfg->kd_filter, dt);
    pid->ki = cfg->ki * dt;
    const float softstart_ramp = 0.2f;  // in seconds
    pid->softstart_ramp_step_size = dt / softstart_ramp;
}

void pid_update(PID *pid, const IMUData *imu, const MotorData *mot, const CfgPid *cfg, float setpoint) {
    // Prepare Brake Scaling (ramp scale values as needed for smooth transitions)
    // if (mot->erpm_abs < 500) {
    //     // All scaling should roll back to 1.0x when near a stop for a smooth stand-still
    //     // and back-forth transition
    //     pid->kp_brake_scale = 0.01f + 0.99f * pid->kp_brake_scale;
    //     pid->kd_brake_scale = 0.01f + 0.99f * pid->kd_brake_scale;
    //     pid->kp_accel_scale = 0.01f + 0.99f * pid->kp_accel_scale;
    //     pid->kd_accel_scale = 0.01f + 0.99f * pid->kd_accel_scale;
    // } else if (mot->erpm > 0) {
    //     // Once rolling forward, brakes should transition to scaled values
    //     pid->kp_brake_scale = 0.01f * cfg->kp_brake + 0.99f * pid->kp_brake_scale;
    //     pid->kd_brake_scale = 0.01f * cfg->kd_brake + 0.99f * pid->kd_brake_scale;
    //     pid->kp_accel_scale = 0.01f + 0.99f * pid->kp_accel_scale;
    //     pid->kd_accel_scale = 0.01f + 0.99f * pid->kd_accel_scale;
    // } else {
    //     // Once rolling backward, the NEW brakes (we will use kp_accel) should transition to
    //     // scaled values
    //     pid->kp_brake_scale = 0.01f + 0.99f * pid->kp_brake_scale;
    //     pid->kd_brake_scale = 0.01f + 0.99f * pid->kd_brake_scale;
    //     pid->kp_accel_scale = 0.01f * cfg->kp_brake + 0.99f * pid->kp_accel_scale;
    //     pid->kd_accel_scale = 0.01f * cfg->kd_brake + 0.99f * pid->kd_accel_scale;
    // }

    const float brake_scale_factor = clamp(fabsf(mot->erpm_smooth) * 0.001f, 0.0f, 1.0f);
    const float kp_brake_scale = 1.0f + (cfg->kp_brake - 1.0f) * brake_scale_factor;
    const float kd_brake_scale = 1.0f + (cfg->kd_brake - 1.0f) * brake_scale_factor;

    const float pitch_offset = setpoint - imu->pitch_balance;
    const int8_t direction = sign(mot->erpm_smooth);

    // PROPORTIONAL
    // float kp_scale;
    // if (pitch_offset < 0.0f) {
    //     kp_scale = cfg->kp * pid->kp_brake_scale;
    // } else {
    //     kp_scale = cfg->kp * pid->kp_accel_scale;
    // }
    // pid->proportional = pitch_offset * kp_scale;

    float kp_scale = cfg->kp;
    if (sign(pitch_offset) != direction) {
        kp_scale *= kp_brake_scale;
    }
    pid->proportional = pitch_offset * kp_scale;
    // TODO expo

    // INTEGRAL
    pid->integral += pitch_offset * pid->ki;
    if (cfg->ki_limit > 0.0f && fabsf(pid->integral) > cfg->ki_limit) {
        pid->integral = cfg->ki_limit * sign(pid->integral);
    }
    // Quickly ramp down integral component during reverse stop
    // if (d->state.sat == SAT_REVERSESTOP) {
    //     pid->integral = pid->integral * 0.9;
    // }

    // DERIVATIVE
    filter_ema(&pid->pitch_rate_filtered, -imu->gyro[1], pid->pitch_rate_alpha); 
    const float rate_prop = pid->pitch_rate_filtered;
    // const float rate_prop = -imu->gyro[1];

    float kd_scale = cfg->kd;
    if (sign(rate_prop) != direction) {
        kd_scale *= kd_brake_scale;
    }
    pid->rate_p = rate_prop * kd_scale;

    // float rate_p_scale;
    // if (rate_prop < 0.0f) {
    //     rate_p_scale = cfg->kd * pid->kd_brake_scale;
    // } else {
    //     rate_p_scale = cfg->kd * pid->kd_accel_scale;
    // }
    // pid->rate_p = rate_prop * rate_p_scale;

    // AGGREGATE
    float new_pid_value = pid->proportional + pid->integral + pid->rate_p;
    // TODO speed boost

    // SOFT START
    if (pid->softstart_pid_limit < 1.0f) {
        new_pid_value *= pid->softstart_pid_limit;
        const float limit_new = pid->softstart_pid_limit + pid->softstart_ramp_step_size;
        pid->softstart_pid_limit = clamp(limit_new, 0.0f, 1.0f);
    }
    // if (pid->softstart_pid_limit < mot->current_max) {
    //     pid->rate_p = fminf(fabs(pid->rate_p), pid->softstart_pid_limit) * sign(pid->rate_p);
    //     pid->softstart_pid_limit += pid->softstart_ramp_step_size;
    // }

    // CURRENT LIMITING
    float current_limit = mot->braking ? mot->current_min : mot->current_max;
    new_pid_value = clamp_sym(new_pid_value, current_limit);
    // TODO soft max?

    filter_ema(&pid->pid_value, new_pid_value, 0.2f); 
}
