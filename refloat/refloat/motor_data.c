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

#include "motor_data.h"

#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

void motor_data_init(MotorData *m) {
    m->erpm_filtered = 0.0f;
    m->erpm_smooth = 0.0f;
    m->erpm_last = 0.0f;
    m->acceleration = 0.0f;
    m->accel_clamped = 0.0f;
    m->duty_smooth = 0.0f;
    m->current_filtered = 0.0f;
}

// void motor_data_reset(MotorData *m, float cooldown_alpha) {
//     // m->erpm_filtered = 0.0f;
//     filter_ema(&m->erpm_filtered, 0.0f, cooldown_alpha);
//     // m->erpm_smooth = 0.0f;
//     filter_ema(&m->erpm_smooth, 0.0f, cooldown_alpha);

//     // m->erpm_last = 0.0f;
//     filter_ema(&m->erpm_last, 0.0f, cooldown_alpha);
//     // m->acceleration = 0.0f;
//     filter_ema(&m->acceleration, 0.0f, cooldown_alpha);
//     // m->accel_clamped = 0.0f;
//     filter_ema(&m->accel_clamped, 0.0f, cooldown_alpha);

//     // m->duty_smooth = 0.0f;
//     filter_ema(&m->duty_smooth, 0.0f, cooldown_alpha);

//     // biquad_reset(&m->atr_current_biquad);
//     // m->current_filtered = 0.0f;
//     filter_ema(&m->current_filtered, 0.0f, cooldown_alpha);
// }

void motor_data_configure(MotorData *m, const CfgTune *cfg, float dt) {
    m->atr_filter_alpha = half_time_to_alpha(cfg->atr.filter, dt);
    m->erpm_filter_alpha = half_time_to_alpha(0.2f, dt);

    // float frequency = cfg->atr.filter * dt;
    // if (frequency > 0) {
    //     biquad_configure(&m->atr_current_biquad, BQ_LOWPASS, frequency);
    //     m->atr_filter_enabled = true;
    // } else {
    //     m->atr_filter_enabled = false;
    // }

    m->current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    // min current is a positive value here!
    m->current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
}

void motor_data_update(MotorData *m, uint16_t frequency) {
    m->erpm = VESC_IF->mc_get_rpm();
    m->erpm_filtered = m->erpm_filtered * 0.9f + m->erpm * 0.1f;
    m->erpm_abs = fabsf(m->erpm_filtered);
    m->erpm_sign = sign(m->erpm_filtered);
    filter_ema(&m->erpm_smooth, m->erpm, m->erpm_filter_alpha);
    m->erpm_abs_10k = fabsf(m->erpm_smooth) * 0.0001f;

    const float accel_raw = (m->erpm - m->erpm_last) * frequency;  // ERPM/s
    const float accel_raw_clamped = clamp_sym(accel_raw, 4000.0f);

    filter_ema(&m->acceleration, accel_raw, m->atr_filter_alpha);
    filter_ema(&m->accel_clamped, accel_raw_clamped, m->atr_filter_alpha);
    m->erpm_last = m->erpm;

    m->current = VESC_IF->mc_get_tot_current_directional_filtered();
    filter_ema(&m->current_filtered, m->current, m->atr_filter_alpha);

    // if (m->atr_filter_enabled) {
    //     m->atr_filtered_current = biquad_process(&m->atr_current_biquad, m->current);
    // } else {
    //     m->atr_filtered_current = m->current;
    // }

    m->braking = m->erpm_abs > 250 && sign(m->current) != m->erpm_sign;

    m->duty_cycle = fabsf(VESC_IF->mc_get_duty_cycle_now());
    m->duty_smooth = m->duty_smooth * 0.9f + m->duty_cycle * 0.1f;
}
