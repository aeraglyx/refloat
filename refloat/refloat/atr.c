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

#include "atr.h"
#include "utils.h"

#include <math.h>

void atr_reset(ATR *atr) {
    atr->target = 0.0f;
    atr->speed = 0.0f;
    atr->interpolated = 0.0f;

    atr->accel_diff = 0.0f;
    atr->debug = 0.0f;
}

void atr_configure(ATR *atr, const RefloatConfig *cfg) {
    // atr->speed_scaled = cfg->atr_speed / cfg->hertz;
    // atr->speed_max_scaled = cfg->atr_speed_max_on / cfg->hertz;
}

void atr_update(ATR *atr, const MotorData *mot, const RefloatConfig *cfg) {
    const float amp_offset = cfg->atr_amp_offset_per_erpm * mot->erpm_smooth;
    const float accel_expected = (mot->current_filtered - amp_offset) / cfg->atr_amps_accel_ratio;

    const float accel_diff_raw = accel_expected - mot->accel_clamped;
    const float accel_diff_half_time = 0.15f * exp2f(-0.003f * mot->erpm_smooth);
    smooth_value(&atr->accel_diff, accel_diff_raw, accel_diff_half_time, cfg->hertz);

    const bool uphill = sign(atr->accel_diff) == sign(mot->erpm_smooth);
    float strength = uphill ? cfg->atr_strength_up : cfg->atr_strength_down;
    const float speed_boost = powf(cfg->atr_strength_boost, mot->erpm_abs_10k);
    strength *= speed_boost;

    atr->target = atr->accel_diff * strength;
    dead_zonef(&atr->target, cfg->atr_threshold);
    clamp_sym(&atr->target, cfg->atr_angle_limit);
    atr->target = clamp(atr->target, -cfg->atr_angle_limit, cfg->atr_angle_limit);
    
    const float response_boost = powf(cfg->atr_speed_boost, mot->erpm_abs_10k);
    const float speed_k = cfg->atr_speed * response_boost;
    const float offset = atr->target - atr->interpolated;
    atr->speed = offset * speed_k;
    clamp_sym(&atr->speed, cfg->atr_speed_max_on);

    atr->interpolated *= atr->speed / cfg->hertz;
}

void atr_winddown(ATR *atr) {
    atr->interpolated *= 0.995f;
    atr->target *= 0.99f;
}

// static void get_wheelslip_probability(MotorData *mot, const RefloatConfig *cfg) {
// 	float accel_factor = cfg->atr_amps_accel_ratio;
// 	float accel_expected = mot->atr_filtered_current / accel_factor;
// 	float accel_diff = mot->acceleration - accel_expected;
    
//     const float wheelslip_start = 4.0f;
//     const float wheelslip_end = 8.0f;
//     float is_wheelslip = (fabs(accel_diff) - wheelslip_start) / (wheelslip_end - wheelslip_start);
//  is_wheelslip = clamp(is_wheelslip, 0.0f, 1.0f);

//     const float freespin_start = 16000.0f;
//     const float freespin_end = 20000.0f;
//     float is_freespin = (mot->erpm_abs - freespin_start) / (freespin_end - freespin_start);
//  is_freespin = clamp(is_freespin, 0.0f, 1.0f);

//     mot->wheelslip_prob = fmaxf(is_wheelslip, is_freespin);
// }
