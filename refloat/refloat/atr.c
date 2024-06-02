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
    filter_ema(&atr->interpolated, 0.0f, cooldown_alpha);
    filter_ema(&atr->amp_diff, 0.0f, cooldown_alpha);
}

void atr_configure(ATR *atr, const CfgAtr *cfg) {
}

void atr_update(ATR *atr, const MotorData *mot, const CfgAtr *cfg, float dt) {
    const float amp_offset_lerp = clamp_sym(mot->speed_smooth, 1.0f);
    const float amp_offset_constant = cfg->amp_offset_constant * amp_offset_lerp;
    const float amp_offset_variable = cfg->amp_offset_variable * mot->speed_smooth;
    const float amp_offset = amp_offset_constant + amp_offset_variable;

    const float amps = mot->current_filtered - amp_offset;
    const float amps_expected = mot->accel_clamped * cfg->amps_accel_ratio;
    const float amp_diff_raw = amps - amps_expected;

    const float half_time = 0.15f * exp2f(-3.0f * fabsf(mot->speed_smooth));
    const float alpha = half_time_to_alpha(half_time, dt);
    filter_ema(&atr->amp_diff, amp_diff_raw, alpha);

    atr->target = atr->amp_diff;
    dead_zonef(&atr->target, cfg->threshold);

    const bool uphill = sign(atr->amp_diff) == sign(mot->speed_smooth);
    const float strength = uphill ? cfg->strength_up : cfg->strength_down;
    atr->target *= strength;

    const float strength_boost = powf(cfg->strength_boost, mot->erpm_abs_10k);
    atr->target *= strength_boost;

    atr->target = clamp_sym(atr->target, cfg->angle_limit);
    
    const float response_boost = powf(cfg->speed_boost, mot->erpm_abs_10k);
    const float speed_k = cfg->speed * response_boost;

    const float offset = atr->target - atr->interpolated;
    atr->speed = clamp_sym(offset * speed_k, cfg->speed_max);

    atr->interpolated += atr->speed * dt;
}

void atr_winddown(ATR *atr) {
    atr->interpolated *= 0.995f;
    atr->amp_diff *= 0.995f;
}
