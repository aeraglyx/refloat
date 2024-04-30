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

#pragma once

#include "conf/datatypes.h"
#include "motor_data.h"
#include "imu_data.h"

typedef struct {
    float pid_value;

    float proportional;
    float integral;
    float rate_p;

    float kp_brake_scale;
    float kp2_brake_scale;
    float kp_accel_scale;
    float kp2_accel_scale;

    float ki;

    float softstart_ramp_step_size;
    float softstart_pid_limit;
} PID;

void pid_reset(PID *pid);

void pid_configure(PID *pid, const RefloatConfig *config);

void pid_update(PID *pid, const IMUData *imu, const MotorData *motor, const RefloatConfig *config, float setpoint);
