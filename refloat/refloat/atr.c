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
    atr->accel_diff = 0.0f;
    atr->target = 0.0f;
    atr->interpolated = 0.0f;
    atr->step_smooth = 0.0f;
    atr->braketilt_target = 0.0f;
    atr->braketilt_interpolated = 0.0f;
}

void atr_configure(ATR *atr, const RefloatConfig *cfg) {
    atr->step_size_on = cfg->atr_on_speed / cfg->hertz;
    atr->step_size_off = cfg->atr_off_speed / cfg->hertz;

    if (cfg->braketilt_strength == 0.0f) {
        atr->braketilt_factor = 0.0f;
    } else {
        atr->braketilt_factor = -(0.5f + (20 - cfg->braketilt_strength) / 5.0f);
    }
}

// static void get_wheelslip_probability(MotorData *mot, const RefloatConfig *cfg) {
// 	float accel_factor = cfg->atr_amps_accel_ratio;
// 	float accel_expected = mot->atr_filtered_current / accel_factor;
// 	float accel_diff = mot->acceleration - accel_expected;
    
//     const float wheelslip_start = 4.0f;
//     const float wheelslip_end = 8.0f;
//     float is_wheelslip = (fabs(accel_diff) - wheelslip_start) / (wheelslip_end - wheelslip_start);
// 	is_wheelslip = clampf(is_wheelslip, 0.0f, 1.0f);

//     const float freespin_start = 16000.0f;
//     const float freespin_end = 20000.0f;
//     float is_freespin = (mot->abs_erpm - freespin_start) / (freespin_end - freespin_start);
// 	is_freespin = clampf(is_freespin, 0.0f, 1.0f);

//     mot->wheelslip_prob = fmaxf(is_wheelslip, is_freespin);
// }

static void atr_update(ATR *atr, const MotorData *mot, const RefloatConfig *cfg) {
    float atr_threshold = mot->braking ? cfg->atr_threshold_down : cfg->atr_threshold_up;
    float accel_factor = cfg->atr_amps_accel_ratio;
    float offset_per_erpm = cfg->atr_amp_offset_per_erpm;
    // float measured_acc = clampf(mot->acceleration, -5.0f, 5.0f);

    float amp_offset = offset_per_erpm * mot->erpm_smooth;
    float accel_expected = (mot->current_filtered - amp_offset) / accel_factor;

    float accel_diff_raw = accel_expected - mot->accel_clamped;
    float accel_diff_half_time = 0.15f * exp2f(-0.003f * mot->erpm_smooth);
    smooth_value(&atr->accel_diff, accel_diff_raw, accel_diff_half_time, cfg->hertz);
    // atr->accel_diff = 0.95f * atr->accel_diff + 0.05f * accel_diff_raw;

    float uphill = sign(atr->accel_diff) == sign(mot->erpm_smooth);
    float strength = uphill ? cfg->atr_strength_up : cfg->atr_strength_down;
    float speed_boost = powf(cfg->atr_speed_boost, fabsf(mot->erpm_smooth) * 0.0001f);

    float new_atr_target = atr->accel_diff * strength * speed_boost;

    dead_zonef(&new_atr_target, atr_threshold);
    angle_limitf(&new_atr_target, cfg->atr_angle_limit);
    atr->target = new_atr_target;

    float step = set_step(atr->interpolated, atr->target, atr->step_size_on, atr->step_size_off);
    float response_boost = powf(cfg->atr_response_boost, fabsf(mot->erpm_smooth) * 0.0001f);
    step *= response_boost;

    float ramp = cfg->atr_ramp;
    float half_time = ramp * 0.5f;

    float step_new = rate_limit_v04(atr->interpolated, atr->target, step, ramp);
    smooth_value(&atr->step_smooth, step_new, half_time, cfg->hertz);
    atr->interpolated += atr->step_smooth;
}

static void braketilt_update(
    ATR *atr, const MotorData *mot, const RefloatConfig *cfg, float proportional
) {
    // braking also should cause setpoint change lift, causing a delayed lingering nose lift
    if (atr->braketilt_factor < 0 && mot->braking && mot->abs_erpm > 2000) {
        // negative currents alone don't necessarily consitute active braking, look at proportional:
        if (sign(proportional) != mot->erpm_sign) {
            float downhill_damper = 1;
            // if we're braking on a downhill we don't want braking to lift the setpoint quite as
            // much
            if ((mot->erpm > 1000 && atr->accel_diff < -1) ||
                (mot->erpm < -1000 && atr->accel_diff > 1)) {
                downhill_damper += fabsf(atr->accel_diff) / 2;
            }
            atr->braketilt_target = proportional / atr->braketilt_factor / downhill_damper;
            if (downhill_damper > 2) {
                // steep downhills, we don't enable this feature at all!
                atr->braketilt_target = 0;
            }
        }
    } else {
        atr->braketilt_target = 0;
    }

    float braketilt_step_size = atr->step_size_off / cfg->braketilt_lingering;
    if (fabsf(atr->braketilt_target) > fabsf(atr->braketilt_interpolated)) {
        braketilt_step_size = atr->step_size_on * 1.5;
    } else if (mot->abs_erpm < 800) {
        braketilt_step_size = atr->step_size_on;
    }

    if (mot->abs_erpm < 500) {
        braketilt_step_size /= 2.0f;
    }

    rate_limitf(&atr->braketilt_interpolated, atr->braketilt_target, braketilt_step_size);
}

void atr_and_braketilt_update(
    ATR *atr, const MotorData *mot, const RefloatConfig *cfg, float proportional
) {
    atr_update(atr, mot, cfg);
    braketilt_update(atr, mot, cfg, proportional);
}

void atr_and_braketilt_winddown(ATR *atr) {
    atr->interpolated *= 0.995f;
    atr->target *= 0.99f;
    atr->braketilt_interpolated *= 0.995f;
    atr->braketilt_target *= 0.99f;
}
