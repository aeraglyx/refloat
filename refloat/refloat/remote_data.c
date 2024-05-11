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

#include "remote_data.h"

#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

void remote_data_reset(RemoteData *remote) {
    // TODO should initialize to throttle?
    remote->throttle_filtered = 0.0f;
}

void remote_data_configure(RemoteData *remote, const RefloatConfig *cfg) {
    remote->throttle_filter_alpha = half_time_to_alpha(cfg->inputtilt_filter, cfg->hertz);
}

void remote_data_update(RemoteData *remote, const RefloatConfig *cfg) {
    bool remote_connected = false;
    float servo_val = 0.0f;

    switch (d->float_conf.inputtilt_remote_type) {
        case (INPUTTILT_PPM):
            servo_val = VESC_IF->get_ppm();
            remote_connected = VESC_IF->get_ppm_age() < 1;
            break;
        case (INPUTTILT_UART): {
            remote_state remote = VESC_IF->get_remote_state();
            servo_val = remote.js_y;
            remote_connected = remote.age_s < 1;
            break;
        }
        case (INPUTTILT_NONE):
            break;
    }

    if (remote_connected) {
        float deadband = cfg->inputtilt_deadband;
        if (fabsf(servo_val) < deadband) {
            servo_val = 0.0f;
        } else {
            servo_val = sign(servo_val) * (fabsf(servo_val) - deadband) / (1.0f - deadband);
        }

        if (cfg->inputtilt_invert_throttle) {
            servo_val *= -1.0f;
        }
    }

    remote->throttle = servo_val;
    filter_ema(&remote->throttle_filtered, remote->throttle, remote->throttle_filter_alpha);
}
