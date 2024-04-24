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

void pid_reset(PID *pid) {
    pid->pid_value = 0.0f;

    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->rate_p = 0.0f;

    pid->kp_brake_scale = 1.0f;
    pid->kp2_brake_scale = 1.0f;
    pid->kp_accel_scale = 1.0f;
    pid->kp2_accel_scale = 1.0f;

    pid->softstart_pid_limit = 0.0f;
}

void pid_configure(PID *pid, const RefloatConfig *cfg) {
    pid->softstart_ramp_step_size = 200.0f / cfg->hertz;
}

void pid_update(PID *pid, const IMUData *imu, const MotorData *mot, const RefloatConfig *cfg, float setpoint) {
    // Prepare Brake Scaling (ramp scale values as needed for smooth transitions)
    if (mot->abs_erpm < 500) {
        // All scaling should roll back to 1.0x when near a stop for a smooth stand-still
        // and back-forth transition
        pid->kp_brake_scale = 0.01 + 0.99 * pid->kp_brake_scale;
        pid->kp2_brake_scale = 0.01 + 0.99 * pid->kp2_brake_scale;
        pid->kp_accel_scale = 0.01 + 0.99 * pid->kp_accel_scale;
        pid->kp2_accel_scale = 0.01 + 0.99 * pid->kp2_accel_scale;
    } else if (mot->erpm > 0) {
        // Once rolling forward, brakes should transition to scaled values
        pid->kp_brake_scale = 0.01 * cfg->kp_brake + 0.99 * pid->kp_brake_scale;
        pid->kp2_brake_scale = 0.01 * cfg->kp2_brake + 0.99 * pid->kp2_brake_scale;
        pid->kp_accel_scale = 0.01 + 0.99 * pid->kp_accel_scale;
        pid->kp2_accel_scale = 0.01 + 0.99 * pid->kp2_accel_scale;
    } else {
        // Once rolling backward, the NEW brakes (we will use kp_accel) should transition to
        // scaled values
        pid->kp_brake_scale = 0.01 + 0.99 * pid->kp_brake_scale;
        pid->kp2_brake_scale = 0.01 + 0.99 * pid->kp2_brake_scale;
        pid->kp_accel_scale = 0.01 * cfg->kp_brake + 0.99 * pid->kp_accel_scale;
        pid->kp2_accel_scale = 0.01 * cfg->kp2_brake + 0.99 * pid->kp2_accel_scale;
    }

    pid->proportional = setpoint - imu->pitch_balance;
    pid->integral = pid->integral + pid->proportional * cfg->ki;

    if (cfg->ki_limit > 0 && fabsf(pid->integral) > cfg->ki_limit) {
        pid->integral = cfg->ki_limit * sign(pid->integral);
    }
    // Quickly ramp down integral component during reverse stop
    // if (d->state.sat == SAT_REVERSESTOP) {
    //     pid->integral = pid->integral * 0.9;
    // }

    // Apply P Brake Scaling
    float scaled_kp;
    if (pid->proportional < 0) {
        scaled_kp = cfg->kp * pid->kp_brake_scale;
    } else {
        scaled_kp = cfg->kp * pid->kp_accel_scale;
    }

    float new_pid_value = scaled_kp * pid->proportional + pid->integral;

    // Rate P (Angle + Rate, rather than Angle-Rate Cascading)
    float rate_prop = -imu->gyro[1];

    float scaled_rate_p;
    if (rate_prop < 0) {
        scaled_rate_p = cfg->kp2 * pid->kp2_brake_scale;
    } else {
        scaled_rate_p = cfg->kp2 * pid->kp2_accel_scale;
    }

    pid->rate_p = scaled_rate_p * rate_prop;

    // Braketilt excluded to allow for soft brakes that strengthen when near tail-drag
    // float true_proportional = d->setpoint - d->atr.braketilt_interpolated - d->imu.pitch;
    // float abs_proportional = fabsf(true_proportional);

    if (pid->softstart_pid_limit < d->mc_current_max) {
        pid->rate_p = fminf(fabs(pid->rate_p), pid->softstart_pid_limit) * sign(pid->rate_p);
        pid->softstart_pid_limit += pid->softstart_ramp_step_size;
    }

    new_pid_value += pid->rate_p;

    // Current Limiting!
    float current_limit = mot->braking ? d->mc_current_min : d->mc_current_max;
    if (fabsf(new_pid_value) > current_limit) {
        new_pid_value = sign(new_pid_value) * current_limit;
    }

    pid->pid_value = pid->pid_value * 0.8 + new_pid_value * 0.2;
}
