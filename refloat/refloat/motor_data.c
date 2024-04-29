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

void motor_data_reset(MotorData *m) {
    m->erpm_filtered = 0.0f;
    m->erpm_smooth = 0.0f;

    m->last_erpm = 0.0f;
    m->acceleration = 0.0f;
    m->accel_clamped = 0.0f;

    m->duty_smooth = 0.0f;

    biquad_reset(&m->atr_current_biquad);
    m->current_filtered = 0.0f;
}

void motor_data_configure(MotorData *m, RefloatConfig *cfg) {
    m->filter_half_time = 1.0f / cfg->atr_filter;

    float frequency = cfg->atr_filter / cfg->hertz;
    if (frequency > 0) {
        biquad_configure(&m->atr_current_biquad, BQ_LOWPASS, frequency);
        m->atr_filter_enabled = true;
    } else {
        m->atr_filter_enabled = false;
    }

    m->current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    // min current is a positive value here!
    m->current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
}

void motor_data_update(MotorData *m) {
    m->erpm = VESC_IF->mc_get_rpm();
    m->erpm_filtered = m->erpm_filtered * 0.9f + m->erpm * 0.1f;
    m->erpm_abs = fabsf(m->erpm_filtered);
    m->erpm_sign = sign(m->erpm_filtered);
    m->erpm_smooth = m->erpm_smooth * 0.996f + m->erpm * 0.004f;
    m->erpm_abs_10k = fabsf(m->erpm_smooth) * 0.0001f;

    float current_acceleration = m->erpm - m->last_erpm;
    float current_accel_clamped = clampf(current_acceleration, -5.0f, 5.0f);

    smooth_value(&m->acceleration, current_acceleration, m->filter_half_time, 800);
    smooth_value(&m->accel_clamped, current_accel_clamped, m->filter_half_time, 800);
    // m->acceleration = m->acceleration * 0.98f + current_acceleration * 0.02f;
    // m->accel_clamped = m->accel_clamped * 0.98f + current_accel_clamped * 0.02f;
    m->last_erpm = m->erpm;

    m->current = VESC_IF->mc_get_tot_current_directional_filtered();
    smooth_value(&m->current_filtered, m->current, m->filter_half_time, 800);

    if (m->atr_filter_enabled) {
        m->atr_filtered_current = biquad_process(&m->atr_current_biquad, m->current);
    } else {
        m->atr_filtered_current = m->current;
    }

    m->braking = m->erpm_abs > 250 && sign(m->current) != m->erpm_sign;
    // m->gas_factor = sigmoid_norm(m->current * sigmoid(m->erpm_filtered, 500), 5.0f);
    // m->braking_factor = sigmoid(m->current * m->erpm_sign, 5.0f);

    m->duty_cycle = fabsf(VESC_IF->mc_get_duty_cycle_now());
    m->duty_smooth = m->duty_smooth * 0.9f + m->duty_cycle * 0.1f;
    // smooth_value(&m->duty_smooth, duty_cycle, 0.01f, 800);
}
