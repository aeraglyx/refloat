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
}

void haptic_buzz_configure(HapticBuzz *data, const CfgWarnings *cfg, float dt) {
    data->step = cfg->buzz_frequency * dt;
}


static float sin_approx(float t) {
    const bool second_half = t > 0.5f;
    const float x = (second_half) ? t - 0.5f : t;
    float sin = 20.0f / (16.0f * x * x - 8.0f * x + 5.0f) - 4.0f;
    if (second_half) {
        sin *= -1.0f;
    }
    return sin;
}

void haptic_buzz_update(
    HapticBuzz *data,
    const Warnings *warnings,
    const CfgWarnings *cfg,
    const MotorData *mot
) {
    data->t += data->step;
    data->t = data->t - (int)data->t;
    // const float strength = fabsf(mot->speed_smooth);
    data->buzz_output = sin_approx(data->t) * cfg->buzz_strength;
    data->buzz_output *= warnings->factor;

    float current_time = VESC_IF->system_time();
    float time = current_time * cfg->buzz_speed;
    float x = time - (long)time;
    float beep_target = (x < 0.5f) ? 1.0f : 0.0f;
    data->buzz_output *= beep_target;
}
