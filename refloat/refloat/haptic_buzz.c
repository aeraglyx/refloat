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

#include "haptic_buzz.h"
#include "utils.h"
#include "state.h"

#include <math.h>

void haptic_buzz_reset(HapticBuzz *data) {
    data->buzz_output = 0.0f;
    data->t = 0.0f;
    data->amplitude = 0.0f;
}

void haptic_buzz_configure(HapticBuzz *data, const CfgWarnings *cfg, float dt) {
    data->step = cfg->buzz_frequency * dt;
}

static float sin_scaled(float t) {
    // based on Bhaskara I's sine approximation
    // valid for 0 <= t <= 1, outputs -1 to 1
    const bool second_half = t > 0.5f;
    const float x = (second_half) ? t - 0.5f : t;
    float sin = 20.0f / (16.0f * x * x - 8.0f * x + 5.0f) - 4.0f;
    if (second_half) {
        sin *= -1.0f;
    }
    return sin;
}

static BuzzType get_buzz_type(WarningReason warning_type) {
    switch (warning_type) {
        case WARNING_NONE:
            return BUZZ_NONE;
        case WARNING_DEBUG:
            return BUZZ_FULL;
        case WARNING_SENSORS:
            return BUZZ_FULL;
        case WARNING_DUTY:
            return BUZZ_FAST;
        default:
            return BUZZ_SLOW;
    }
}

static bool get_beep_target(BuzzType buzz_type, float speed) {
    bool beep;
    if (buzz_type == BUZZ_NONE) {
        beep = false;
    } else if (buzz_type == BUZZ_FULL) {
        beep = true;
    } else {
        if (buzz_type == BUZZ_SLOW) {
            speed *= 0.25f;
        }
        const float current_time = VESC_IF->system_time();
        const float time = current_time * speed;
        const float x = time - (long)time;
        beep = (x < 0.5f);
    }
    return beep;
}

void haptic_buzz_update(
    HapticBuzz *data,
    const Warnings *warnings,
    const CfgWarnings *cfg,
    const MotorData *mot
) {
    const BuzzType buzz_type = get_buzz_type(warnings->reason);
    const bool beep_target = get_beep_target(buzz_type, cfg->buzz_speed);

    const float strength_variable = fabsf(mot->speed_smooth) * cfg->buzz_strength_variable;
    const float strength = cfg->buzz_strength + strength_variable;
    const float amplitude = (float)beep_target * warnings->factor * strength;

    // Limit amplitude's rate of change (just in case),
    // we don't want to induce accidental current overshoots under load.
    rate_limitf(&data->amplitude, amplitude, 2.5f);

    if (data->amplitude == 0.0f) {
        data->buzz_output = 0.0f;
    } else {
        data->t += data->step;
        data->t = data->t - (int)data->t;
        const float wave = sin_scaled(data->t);
        data->buzz_output = wave * data->amplitude;
    }
}
