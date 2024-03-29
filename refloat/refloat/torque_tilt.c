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

#include "torque_tilt.h"

#include "utils.h"

#include <math.h>

void torque_tilt_reset(TorqueTilt *tt) {
    tt->offset = 0;
}

void torque_tilt_configure(TorqueTilt *tt, const RefloatConfig *cfg) {
    tt->on_step_size = cfg->torquetilt_on_speed / cfg->hertz;
    tt->off_step_size = cfg->torquetilt_off_speed / cfg->hertz;
}

void torque_tilt_update(TorqueTilt *tt, const MotorData *mot, const RefloatConfig *cfg) {
    float strength =
        mot->braking ? cfg->torquetilt_strength_regen : cfg->torquetilt_strength;
    
    // float strength = remap(
    //     mot->gas_factor, cfg->torquetilt_strength_regen, cfg->torquetilt_strength
    // );
    // float strength = cfg->torquetilt_strength;

    // float accel_factor = remap(
    //     mot->gas_factor, cfg->atr_amps_decel_ratio, cfg->atr_amps_accel_ratio
    // );
    float accel_factor = cfg->atr_amps_accel_ratio;

    float torque_offset = 0.00022f * mot->erpm_smooth * accel_factor;
	float current_adjusted = mot->atr_filtered_current - torque_offset;

    float target_offset = current_adjusted;
    dead_zonef(&target_offset, cfg->torquetilt_start_current);
    target_offset *= strength;
    angle_limitf(&target_offset, cfg->torquetilt_angle_limit);
    
    // 1.5 should be similar to the old behavior
    // dead_zonef(&tt->offset, cfg->torquetilt_threshold);  // was torquetilt_start_current

    float responsivness = cfg->brkbooster_ramp;

    // if (mot->abs_erpm < 500) {
    //     responsivness /= 2;
    // }

    // limit_speed(&tt->offset, target_offset, responsivness, cfg->torquetilt_on_speed, cfg->hertz);
    rate_limit_v02(&tt->offset, target_offset, tt->on_step_size, responsivness);

    // rate_limitf(&tt->offset, target_offset, step_size);
}

void torque_tilt_winddown(TorqueTilt *tt) {
    tt->offset *= 0.995;
}
