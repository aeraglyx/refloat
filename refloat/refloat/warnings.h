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
#include "footpad_sensor.h"

typedef enum {
    WARNING_NONE = 0,
    WARNING_LV = 1,
    WARNING_HV = 2,
    WARNING_TEMPFET = 3,
    WARNING_TEMPMOT = 4,
    WARNING_CURRENT = 5,
    WARNING_DUTY = 6,
    WARNING_SENSORS = 7,
    WARNING_LOWBATT = 8,
    WARNING_IDLE = 9,
    WARNING_ERROR = 10
} WarningReason;

typedef struct {
    float factor;
    WarningReason reason;

    float duty;
    float hv;
    float lv;
    float temp_fet;
    float temp_mot;
    float sensor;

    float duty_step;
    float hv_step;
    float lv_step;
    float temp_step;

    float mc_max_temp_fet;
    float mc_max_temp_mot;
} Warnings;

void warnings_reset(Warnings *warnings, float cooldown_alpha);

void warnings_configure(Warnings *warnings, float dt);

void warnings_update(
    Warnings *warnings,
    const MotorData *mot,
    const CfgWarnings *cfg,
    const FootpadSensor *sensor
);
