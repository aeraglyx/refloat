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
    atr->speed_boost = 0.0f;
    atr->target = 0.0f;
    atr->interpolated = 0.0f;
    atr->braketilt_target_offset = 0.0f;
    atr->braketilt_offset = 0.0f;
    atr->step_smooth = 0.0f;
}

void atr_configure(ATR *atr, const RefloatConfig *cfg) {
    atr->step_size_on = cfg->atr_on_speed / cfg->hertz;
    atr->step_size_off = cfg->atr_off_speed / cfg->hertz;

    atr->speed_boost_mult = 1.0f / 3000.0f;
    if (fabsf(cfg->atr_speed_boost) > 0.4f) {
        atr->speed_boost_mult = 1.0f / ((fabsf(cfg->atr_speed_boost) - 0.4f) * 5000.0f + 3000.0f);
    }

    if (cfg->braketilt_strength == 0.0f) {
        atr->braketilt_factor = 0.0f;
    } else {
        atr->braketilt_factor = -(0.5f + (20 - cfg->braketilt_strength) / 5.0f);
    }
}

// static void get_wheelslip_probability(MotorData *mot, const RefloatConfig *cfg) {
// 	float accel_factor = cfg->atr_amps_accel_ratio;
// 	float expected_acc = mot->atr_filtered_current / accel_factor;
// 	float accel_diff = mot->acceleration - expected_acc;
    
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
    float offset_per_erpm = cfg->atr_amps_decel_ratio;
    // float measured_acc = clampf(mot->acceleration, -5.0f, 5.0f);

    // float amp_offset = 0.00015f * mot->erpm_smooth * accel_factor;
    // float amps_adjusted = mot->current_filtered - amp_offset;
    float amp_offset = offset_per_erpm * mot->erpm_smooth;
    float expected_acc = (mot->current_filtered - amp_offset) / accel_factor;

    float new_accel_diff = expected_acc - mot->accel_clamped;
    float accel_diff_half_time = 0.15f * exp2f(-0.003f * mot->erpm_smooth);
    smooth_value(&atr->accel_diff, new_accel_diff, accel_diff_half_time, cfg->hertz);
    // atr->accel_diff = 0.95f * atr->accel_diff + 0.05f * new_accel_diff;

    bool forward = mot->erpm_filtered > 0;
    if (mot->abs_erpm < 250 && fabsf(mot->current_filtered) > 30) {
        forward = (expected_acc > 0);
    }

    float atr_strength =
        forward == (atr->accel_diff > 0) ? cfg->atr_strength_up : cfg->atr_strength_down;

    atr->speed_boost = exp2f(cfg->atr_speed_boost * fabsf(mot->erpm_smooth) / 10000);

    float new_atr_target = atr->accel_diff * atr_strength * atr->speed_boost;

    dead_zonef(&new_atr_target, atr_threshold);
    angle_limitf(&new_atr_target, cfg->atr_angle_limit);
    // atr->target = 0.95f * atr->target + 0.05f * new_atr_target;
    atr->target = new_atr_target;

    float ramp = cfg->booster_angle / 10.0f;
    float half_time = cfg->booster_ramp / 50.0f;

    float step = set_step(atr->interpolated, atr->target, atr->step_size_on, atr->step_size_off);
    float response_boost = exp2f(cfg->atr_response_boost * fabsf(mot->erpm_smooth) / 10000);
    step *= response_boost;

    // rate_limit_v02(&atr->interpolated, atr->target, step, ramp);
    // smooth_value(&atr->interpolated, interpolated, half_time, cfg->hertz);
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
            atr->braketilt_target_offset = proportional / atr->braketilt_factor / downhill_damper;
            if (downhill_damper > 2) {
                // steep downhills, we don't enable this feature at all!
                atr->braketilt_target_offset = 0;
            }
        }
    } else {
        atr->braketilt_target_offset = 0;
    }

    float braketilt_step_size = atr->step_size_off / cfg->braketilt_lingering;
    if (fabsf(atr->braketilt_target_offset) > fabsf(atr->braketilt_offset)) {
        braketilt_step_size = atr->step_size_on * 1.5;
    } else if (mot->abs_erpm < 800) {
        braketilt_step_size = atr->step_size_on;
    }

    if (mot->abs_erpm < 500) {
        braketilt_step_size /= 2.0f;
    }

    rate_limitf(&atr->braketilt_offset, atr->braketilt_target_offset, braketilt_step_size);
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
    atr->braketilt_offset *= 0.995f;
    atr->braketilt_target_offset *= 0.99f;
}
