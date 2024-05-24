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

void atr_reset(ATR *atr, float cooldown_alpha) {
    // atr->target = 0.0f;
    // atr->speed = 0.0f;
    // atr->interpolated = 0.0f;
    filter_ema(&atr->interpolated, 0.0f, cooldown_alpha);
    // atr->amp_diff = 0.0f;
    filter_ema(&atr->amp_diff, 0.0f, cooldown_alpha);
    // atr->debug = 0.0f;
    atr->debug = cooldown_alpha;
}

void atr_configure(ATR *atr, const CfgAtr *cfg) {
    atr->accel_amps_ratio = 1.0f / cfg->amps_accel_ratio;
}

void atr_update(ATR *atr, const MotorData *mot, const CfgAtr *cfg, float dt) {
    const float amp_offset_lerp = clamp_sym(mot->erpm_smooth * 0.001f, 1.0f);
    const float amp_offset_constant = cfg->amp_offset_constant * amp_offset_lerp;
    const float amp_offset_variable = cfg->amp_offset_variable * mot->erpm_smooth;
    const float amp_offset = amp_offset_constant + amp_offset_variable;

    const float amps = mot->current_filtered - amp_offset;
    const float amps_expected = mot->accel_clamped * cfg->amps_accel_ratio;
    const float amp_diff_raw = amps - amps_expected;

    const float half_time = 0.15f * exp2f(-0.003f * fabsf(mot->erpm_smooth));
    const float alpha = half_time_to_alpha(half_time, dt);
    filter_ema(&atr->amp_diff, amp_diff_raw, alpha);

    const bool uphill = sign(atr->amp_diff) == sign(mot->erpm_smooth);
    float strength = uphill ? cfg->strength_up : cfg->strength_down;
    const float strength_boost = powf(cfg->strength_boost, mot->erpm_abs_10k);
    strength *= strength_boost;

    atr->target = atr->amp_diff;
    dead_zonef(&atr->target, cfg->threshold);
    atr->target *= strength;
    atr->target = clamp_sym(atr->target, cfg->angle_limit);
    
    const float response_boost = powf(cfg->speed_boost, mot->erpm_abs_10k);
    const float speed_k = cfg->speed * response_boost;
    const float offset = atr->target - atr->interpolated;
    atr->speed = offset * speed_k;
    atr->speed = clamp_sym(atr->speed, cfg->speed_max);

    atr->interpolated += atr->speed * dt;
    // atr->debug = amps;
}

void atr_winddown(ATR *atr) {
    atr->interpolated *= 0.995f;
    atr->amp_diff *= 0.995f;
}

// static void get_wheelslip_probability(MotorData *mot, const RefloatConfig *cfg) {
// 	float accel_factor = cfg->atr_amps_accel_ratio;
// 	float accel_expected = mot->current_filtered / accel_factor;
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
