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

#include "vesc_c_if.h"

#include "warnings.h"
#include "utils.h"
#include "state.h"

#include <math.h>

void warnings_reset(Warnings *warnings, float cooldown_alpha) {
    filter_ema(&warnings->duty, 0.0f, cooldown_alpha);
    filter_ema(&warnings->hv, 0.0f, cooldown_alpha);
    filter_ema(&warnings->lv, 0.0f, cooldown_alpha);
    filter_ema(&warnings->temp_fet, 0.0f, cooldown_alpha);
    filter_ema(&warnings->temp_mot, 0.0f, cooldown_alpha);
}

void warnings_configure(Warnings *warnings, float dt) {
    warnings->mc_max_temp_fet = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_fet_start) - 3;
    warnings->mc_max_temp_mot = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_motor_start) - 3;

    warnings->duty_step = 1.0f * dt;
    warnings->hv_step = 0.25f * dt;
    warnings->lv_step = 0.25f * dt;
    warnings->temp_step = 0.25f * dt;
}

void warnings_update(
    Warnings *warnings,
    const MotorData *mot,
    const CfgWarnings *cfg
) {
    const bool duty_active = mot->duty_cycle > cfg->duty.threshold;
    const float duty_target = duty_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->duty, duty_target, warnings->duty_step);


    const float voltage = VESC_IF->mc_get_input_voltage_filtered();
    const float sag = 0.05f * fabsf(mot->current_filtered);
    const bool is_duty_non_zero = mot->duty_cycle > 0.05f;

    const bool hv_active = voltage > cfg->hv.threshold && is_duty_non_zero;
    const float hv_target = hv_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->hv, hv_target, warnings->hv_step);

    const bool lv_active = voltage + sag < cfg->lv.threshold && is_duty_non_zero;
    const float lv_target = lv_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->lv, lv_target, warnings->lv_step);


    const float temp_fet = VESC_IF->mc_temp_fet_filtered();
    const bool temp_fet_active = temp_fet > warnings->mc_max_temp_fet;
    const float temp_fet_target = temp_fet_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->temp_fet, temp_fet_target, warnings->temp_step);

    const float temp_mot = VESC_IF->mc_temp_motor_filtered();
    const bool temp_mot_active = temp_mot > warnings->mc_max_temp_mot;
    const float temp_mot_target = temp_mot_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->temp_mot, temp_mot_target, warnings->temp_step);
}
