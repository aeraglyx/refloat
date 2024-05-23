// Copyright 2019 - 2022 Mitch Lustig
// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#include "balance_filter.h"
#include "imu_data.h"
#include "motor_data.h"
#include "remote_data.h"

#include "atr.h"
#include "torque_tilt.h"
#include "turn_tilt.h"
#include "speed_tilt.h"
#include "input_tilt.h"

#include "pid.h"

#include "charging.h"
#include "footpad_sensor.h"
#include "lcm.h"
#include "leds.h"
#include "state.h"
#include "utils.h"

#include "conf/buffer.h"
#include "conf/conf_general.h"
#include "conf/confparser.h"
#include "conf/confxml.h"
#include "conf/datatypes.h"

#include <math.h>
#include <string.h>

HEADER

typedef enum {
    BEEP_NONE = 0,
    BEEP_LV = 1,
    BEEP_HV = 2,
    BEEP_TEMPFET = 3,
    BEEP_TEMPMOT = 4,
    BEEP_CURRENT = 5,
    BEEP_DUTY = 6,
    BEEP_SENSORS = 7,
    BEEP_LOWBATT = 8,
    BEEP_IDLE = 9,
    BEEP_ERROR = 10
} BeepReason;


// This is all persistent state of the application, which will be allocated in init. It
// is put here because variables can only be read-only when this program is loaded
// in flash without virtual memory in RAM (as all RAM already is dedicated to the
// main firmware and managed from there). This is probably the main limitation of
// loading applications in runtime, but it is not too bad to work around.
typedef struct {
    lib_thread main_thread;
    lib_thread led_thread;

    RefloatConfig config;

    // Firmware version, passed in from Lisp
    int fw_version_major, fw_version_minor, fw_version_beta;

    State state;

    // Board data
    IMUData imu;
    MotorData motor;
    RemoteData remote;

    float mc_max_temp_fet, mc_max_temp_mot;
    // float mc_current_max, mc_current_min;

    // IMU data for the balancing filter
    BalanceFilterData balance_filter;

    // Tune modifiers
    ATR atr;
    TorqueTilt torque_tilt;
    TurnTilt turn_tilt;
    SpeedTilt speed_tilt;
    InputTilt input_tilt;

    PID pid;

    // Beeper
    int beep_num_left;
    int beep_duration;
    int beep_countdown;
    int beep_reason;
    bool beeper_enabled;

    Leds leds;
    LcmData lcm;

    Charging charging;

    // Config values
    float loop_time;
    uint32_t loop_time_us;

    float current_time;
    float last_time;
    float dt_raw;
    float dt;
    float time_diff;

    float startup_pitch_trickmargin;
    float startup_pitch_tolerance;
    float startup_step_size;

    float tiltback_duty_step_size;
    float tiltback_hv_step_size;
    float tiltback_lv_step_size;
    float tiltback_return_step_size;

    float surge_angle;
    float surge_angle2;
    float surge_angle3;
    float surge_adder;
    bool surge_enable;
    bool duty_beeping;

    float max_duty_with_margin;

    FootpadSensor footpad_sensor;

    float setpoint;
    float setpoint_target;
    float setpoint_target_interpolated;

    float disengage_timer;
    float nag_timer;
    float idle_voltage;
    float fault_angle_pitch_timer, fault_angle_roll_timer;
    float fault_switch_timer, fault_switch_half_timer;
    float brake_timeout;
    float wheelslip_timer, tb_highvoltage_timer;
    float switch_warn_beep_erpm;

    // Feature: Reverse Stop
    float reverse_stop_step_size;
    float reverse_tolerance;
    float reverse_total_erpm;
    float reverse_timer;

    // Odometer
    float odo_timer;
    int odometer_dirty;
    uint64_t odometer;

    // Feature: RC Move (control via app while idle)
    int rc_steps;
    int rc_counter;
    float rc_current_target;
    float rc_current;

} data;

static void brake(data *d);
static void set_current(float current);

const VESC_PIN beeper_pin = VESC_PIN_PPM;

#define EXT_BEEPER_ON() VESC_IF->io_write(beeper_pin, 1)
#define EXT_BEEPER_OFF() VESC_IF->io_write(beeper_pin, 0)

void beeper_init() {
    VESC_IF->io_set_mode(beeper_pin, VESC_PIN_MODE_OUTPUT);
}

void beeper_update(data *d) {
    if (d->beeper_enabled && (d->beep_num_left > 0)) {
        d->beep_countdown--;
        if (d->beep_countdown <= 0) {
            d->beep_countdown = d->beep_duration;
            d->beep_num_left--;
            if (d->beep_num_left & 0x1) {
                EXT_BEEPER_ON();
            } else {
                EXT_BEEPER_OFF();
            }
        }
    }
}

void beeper_enable(data *d, bool enable) {
    d->beeper_enabled = enable;
    if (!enable) {
        EXT_BEEPER_OFF();
    }
}

void beep_alert(data *d, int num_beeps, bool longbeep) {
    if (!d->beeper_enabled) {
        return;
    }
    if (d->beep_num_left == 0) {
        d->beep_num_left = num_beeps * 2 + 1;
        d->beep_duration = longbeep ? 300 : 80;
        d->beep_countdown = d->beep_duration;
    }
}

void beep_off(data *d, bool force) {
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_OFF();
    }
}

void beep_on(data *d, bool force) {
    if (!d->beeper_enabled) {
        return;
    }
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_ON();
    }
}

static void configure(data *d) {
    // Loop time in seconds and microseconds
    d->loop_time = 1.0f / d->config.hardware.esc.frequency;
    d->loop_time_us = 1e6 / d->config.hardware.esc.frequency;

    state_init(&d->state, d->config.disabled);

    lcm_configure(&d->lcm, &d->config.leds);
    motor_data_configure(&d->motor, &d->config.tune, d->loop_time);
    imu_data_configure(&d->imu, &d->config.tune.turn_tilt, d->loop_time);
    remote_data_configure(&d->remote, &d->config.tune.input_tilt, d->loop_time);

    balance_filter_configure(&d->balance_filter, &d->config.tune.balance_filter);
    atr_configure(&d->atr, &d->config.tune.atr);
    // torque_tilt_configure(&d->torque_tilt, &d->config);
    // turn_tilt_configure(&d->turn_tilt, &d->config);
    speed_tilt_configure(&d->speed_tilt, &d->config.tune.speed_tilt);
    // input_tilt_configure(&d->input_tilt, &d->config);
    pid_configure(&d->pid, &d->config.tune.pid, d->loop_time);

    // This timer is used to determine how long the board has been disengaged / idle
    d->disengage_timer = d->current_time;

    d->startup_step_size = d->config.startup_speed * d->loop_time;

    d->tiltback_duty_step_size = d->config.warnings.duty.tiltback_speed * d->loop_time;
    d->tiltback_hv_step_size = d->config.warnings.hv.tiltback_speed * d->loop_time;
    d->tiltback_lv_step_size = d->config.warnings.lv.tiltback_speed * d->loop_time;
    d->tiltback_return_step_size = d->config.warnings.tiltback_return_speed * d->loop_time;

    d->surge_angle = d->config.surge_angle;
    d->surge_angle2 = d->config.surge_angle * 2;
    d->surge_angle3 = d->config.surge_angle * 3;
    d->surge_enable = d->surge_angle > 0;

    // Feature: Dirty Landings
    d->startup_pitch_trickmargin = d->config.startup_dirtylandings_enabled ? 10 : 0;

    // Backwards compatibility hack:
    // If mahony kp from the firmware internal filter is higher than 1, it's
    // the old setup with it being the main balancing filter. In that case, set
    // the kp and acc confidence decay to hardcoded defaults of the former true
    // pitch filter, to preserve the behavior of the old setup in the new one.
    // (Though Mahony KP 0.4 instead of 0.2 is used, as it seems to work better)
    if (VESC_IF->get_cfg_float(CFG_PARAM_IMU_mahony_kp) > 1) {
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_kp, 0.4);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_ki, 0);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_accel_confidence_decay, 0.1);
    }

    d->mc_max_temp_fet = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_fet_start) - 3;
    d->mc_max_temp_mot = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_motor_start) - 3;

    // d->mc_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    // min current is a positive value here!
    // d->mc_current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));

    d->max_duty_with_margin = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty) - 0.1;

    // Feature: Reverse Stop
    d->reverse_tolerance = 50000;
    d->reverse_stop_step_size = 100.0 * d->loop_time;

    // Speed above which to warn users about an impending full switch fault
    d->switch_warn_beep_erpm = d->config.warnings.is_footbeep_enabled ? 2000 : 100000;

    d->beeper_enabled = d->config.hardware.esc.is_beeper_enabled;

    if (d->state.state == STATE_DISABLED) {
        beep_alert(d, 3, false);
    } else {
        beep_alert(d, 1, false);
    }
}

static void reset_vars(data *d) {
    const float time_disengaged = d->current_time - d->disengage_timer;
    const float cooldown_alpha = half_time_to_alpha(0.75f, time_disengaged);

    imu_data_reset(&d->imu, cooldown_alpha);
    motor_data_reset(&d->motor, cooldown_alpha);
    remote_data_reset(&d->remote, cooldown_alpha);

    atr_reset(&d->atr, cooldown_alpha);
    torque_tilt_reset(&d->torque_tilt, cooldown_alpha);
    turn_tilt_reset(&d->turn_tilt, cooldown_alpha);
    speed_tilt_reset(&d->speed_tilt, cooldown_alpha);
    input_tilt_reset(&d->input_tilt, &d->remote, cooldown_alpha);

    pid_reset(&d->pid, &d->config.tune.pid, cooldown_alpha);

    // Set values for startup
    // d->setpoint = d->imu.pitch_balance;
    filter_ema(&d->setpoint, d->imu.pitch_balance, cooldown_alpha);
    // d->setpoint_target_interpolated = d->imu.pitch_balance;
    filter_ema(&d->setpoint_target_interpolated, d->imu.pitch_balance, cooldown_alpha);
    d->setpoint_target = 0.0f;
    
    d->brake_timeout = 0.0f;

    d->startup_pitch_tolerance = d->config.startup_pitch_tolerance;
    d->surge_adder = 0.0f;

    // RC Move:
    d->rc_steps = 0.0f;
    d->rc_current = 0.0f;

    state_engage(&d->state);
}

/**
 * check_odometer: see if we need to write back the odometer during fault state
 */
static void check_odometer(data *d) {
    // Make odometer persistent if we've gone 200m or more
    if (d->odometer_dirty > 0) {
        float stored_odo = VESC_IF->mc_get_odometer();
        if ((stored_odo > d->odometer + 200) || (stored_odo < d->odometer - 10000)) {
            if (d->odometer_dirty == 1) {
                // Wait 10 seconds before writing to avoid writing if immediately continuing to ride
                d->odo_timer = d->current_time;
                d->odometer_dirty++;
            } else if ((d->current_time - d->odo_timer) > 10) {
                VESC_IF->store_backup_data();
                d->odometer = VESC_IF->mc_get_odometer();
                d->odometer_dirty = 0;
            }
        }
    }
}

/**
 *  do_rc_move: perform motor movement while board is idle
 */
static void do_rc_move(data *d) {
    if (d->rc_steps > 0) {
        d->rc_current = d->rc_current * 0.95 + d->rc_current_target * 0.05;
        if (d->motor.erpm_abs > 800) {
            d->rc_current = 0;
        }
        set_current(d->rc_current);
        d->rc_steps--;
        d->rc_counter++;
        if ((d->rc_counter == 500) && (d->rc_current_target > 2)) {
            d->rc_current_target /= 2;
        }
    } else {
        d->rc_counter = 0;

        // Throttle must be greater than 2% (Help mitigate lingering throttle)
        // TODO
        if ((d->config.tune.input_tilt.remote_throttle_current_max > 0) &&
            (d->current_time - d->disengage_timer > d->config.tune.input_tilt.remote_throttle_grace_period) &&
            (fabsf(d->remote.throttle) > 0.02)) {
            // float servo_val = d->throttle_val;
            // servo_val *= (d->config.inputtilt_invert_throttle ? -1.0 : 1.0);
            const float max = d->config.tune.input_tilt.remote_throttle_current_max;
            filter_ema(&d->rc_current, max * d->remote.throttle, 0.05f);
            set_current(d->rc_current);
        } else {
            d->rc_current = 0;
            // Disable output
            brake(d);
        }
    }
}

static float get_setpoint_adjustment_step_size(data *d) {
    switch (d->state.sat) {
    case (SAT_NONE):
        return d->tiltback_return_step_size;
    case (SAT_CENTERING):
        return d->startup_step_size;
    case (SAT_REVERSESTOP):
        return d->reverse_stop_step_size;
    case (SAT_PB_DUTY):
        return d->tiltback_duty_step_size;
    case (SAT_PB_HIGH_VOLTAGE):
    case (SAT_PB_TEMPERATURE):
        return d->tiltback_hv_step_size;
    case (SAT_PB_LOW_VOLTAGE):
        return d->tiltback_lv_step_size;
    default:
        return 0;
    }
}

bool is_sensor_engaged(const data *d) {
    if (d->footpad_sensor.state == FS_BOTH) {
        return true;
    }

    if (d->footpad_sensor.state == FS_LEFT || d->footpad_sensor.state == FS_RIGHT) {
        // 5 seconds after stopping we allow starting with a single sensor (e.g. for jump starts)
        bool is_simple_start =
            d->config.startup_simplestart_enabled && (d->current_time - d->disengage_timer > 5);

        if (d->config.faults.is_posi_enabled || is_simple_start) {
            return true;
        }
    }

    return false;
}

static bool is_orientation_valid(const data *d) {
    const bool is_pitch_valid = fabsf(d->imu.pitch_balance) < d->startup_pitch_tolerance;
    const bool is_roll_valid = fabsf(d->imu.roll) < d->config.startup_roll_tolerance;
    return (is_pitch_valid && is_roll_valid);
}

// Fault checking order does not really matter. From a UX perspective, switch should be before
// angle.
static bool check_faults(data *d) {
    // CfgFaults faults = d->config.faults;

    bool disable_switch_faults = d->config.faults.moving_fault_disabled &&
        // Rolling forward (not backwards!)
        d->motor.erpm > (d->config.faults.switch_half_erpm * 2) &&
        // Not tipped over
        fabsf(d->imu.roll) < 40;

    // Check switch
    // Switch fully open
    if (d->footpad_sensor.state == FS_NONE) {
        if (!disable_switch_faults) {
            if ((1000.0f * (d->current_time - d->fault_switch_timer)) >
                d->config.faults.switch_full_delay) {
                state_stop(&d->state, STOP_SWITCH_FULL);
                return true;
            }
            // low speed (below 6 x half-fault threshold speed):
            else if (
                (d->motor.erpm_abs < d->config.faults.switch_half_erpm * 6) &&
                (1000.0f * (d->current_time - d->fault_switch_timer) >
                    d->config.faults.switch_half_delay)) {
                state_stop(&d->state, STOP_SWITCH_FULL);
                return true;
            }
        }

        if (d->motor.erpm_abs < 200 && fabsf(d->imu.pitch) > 14 &&
            fabsf(d->input_tilt.interpolated) < 30 && sign(d->imu.pitch) == d->motor.erpm_sign) {
            state_stop(&d->state, STOP_QUICKSTOP);
            return true;
        }
    } else {
        d->fault_switch_timer = d->current_time;
    }

    // Feature: Reverse-Stop
    if (d->state.sat == SAT_REVERSESTOP) {
        //  Taking your foot off entirely while reversing? Ignore delays
        if (d->footpad_sensor.state == FS_NONE) {
            state_stop(&d->state, STOP_SWITCH_FULL);
            return true;
        }
        if (fabsf(d->imu.pitch) > 15) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        // Above 10 degrees for a half a second? Switch it off
        if (fabsf(d->imu.pitch) > 10 && d->current_time - d->reverse_timer > .5) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        // Above 5 degrees for a full second? Switch it off
        if (fabsf(d->imu.pitch) > 5 && d->current_time - d->reverse_timer > 1) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        if (d->reverse_total_erpm > d->reverse_tolerance * 3) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        if (fabsf(d->imu.pitch) <= 5) {
            d->reverse_timer = d->current_time;
        }
    }

    // Switch partially open and stopped
    if (!d->config.faults.is_posi_enabled) {
        if (!is_sensor_engaged(d) && d->motor.erpm_abs < d->config.faults.switch_half_erpm) {
            if ((1000.0f * (d->current_time - d->fault_switch_half_timer)) >
                d->config.faults.switch_half_delay) {
                state_stop(&d->state, STOP_SWITCH_HALF);
                return true;
            }
        } else {
            d->fault_switch_half_timer = d->current_time;
        }
    }

    // Check roll angle
    if (fabsf(d->imu.roll) > d->config.faults.roll_threshold) {
        if ((1000.0f * (d->current_time - d->fault_angle_roll_timer)) >
            d->config.faults.roll_delay) {
            state_stop(&d->state, STOP_ROLL);
            return true;
        }
    } else {
        d->fault_angle_roll_timer = d->current_time;
    }

    // Check pitch angle
    if (fabsf(d->imu.pitch) > d->config.faults.pitch_threshold && fabsf(d->input_tilt.interpolated) < 30) {
        if ((1000.0f * (d->current_time - d->fault_angle_pitch_timer)) >
            d->config.faults.pitch_delay) {
            state_stop(&d->state, STOP_PITCH);
            return true;
        }
    } else {
        d->fault_angle_pitch_timer = d->current_time;
    }

    return false;
}

static void calculate_setpoint_target(data *d) {
    float input_voltage = VESC_IF->mc_get_input_voltage_filtered();

    if (input_voltage < d->config.warnings.hv.threshold) {
        d->tb_highvoltage_timer = d->current_time;
    }

    if (d->state.sat == SAT_REVERSESTOP) {
        // accumalete erpms:
        d->reverse_total_erpm += d->motor.erpm;
        if (fabsf(d->reverse_total_erpm) > d->reverse_tolerance) {
            // tilt down by 10 degrees after 50k aggregate erpm
            d->setpoint_target = 10 * (fabsf(d->reverse_total_erpm) - d->reverse_tolerance) / 50000;
        } else {
            if (fabsf(d->reverse_total_erpm) <= d->reverse_tolerance / 2) {
                if (d->motor.erpm >= 0) {
                    d->state.sat = SAT_NONE;
                    d->reverse_total_erpm = 0;
                    d->setpoint_target = 0;
                    d->pid.integral = 0;
                }
            }
        }
    } else if (
        fabsf(d->motor.acceleration) > 12000 &&  // not normal, either wheelslip or wheel getting stuck
        sign(d->motor.acceleration) == d->motor.erpm_sign && d->motor.duty_cycle > 0.3 &&
        d->motor.erpm_abs > 2000)  // acceleration can jump a lot at very low speeds
    {
        d->state.wheelslip = true;
        d->state.sat = SAT_NONE;
        d->wheelslip_timer = d->current_time;
    } else if (d->state.wheelslip) {
        // Remain in wheelslip state for at least 500ms to avoid any overreactions
        if (d->motor.duty_cycle > d->max_duty_with_margin) {
            d->wheelslip_timer = d->current_time;
        } else if (d->current_time - d->wheelslip_timer > 0.2) {
            if (d->motor.duty_cycle < 0.7) {
                // Leave wheelslip state only if duty < 70%
                d->state.wheelslip = false;
            }
        }
        if (d->config.faults.is_reversestop_enabled && (d->motor.erpm < 0)) {
            // the 500ms wheelslip time can cause us to blow past the reverse stop condition!
            d->state.sat = SAT_REVERSESTOP;
            d->reverse_timer = d->current_time;
            d->reverse_total_erpm = 0;
        }
    } else if (d->motor.duty_cycle > d->config.warnings.duty.threshold) {
        if (d->motor.erpm > 0) {
            d->setpoint_target = d->config.warnings.duty.tiltback_angle;
        } else {
            d->setpoint_target = -d->config.warnings.duty.tiltback_angle;
        }
        d->state.sat = SAT_PB_DUTY;
    } else if (d->motor.duty_cycle > 0.05 && input_voltage > d->config.warnings.hv.threshold) {
        d->beep_reason = BEEP_HV;
        beep_alert(d, 3, false);
        if (((d->current_time - d->tb_highvoltage_timer) > .5) ||
            (input_voltage > d->config.warnings.hv.threshold + 1)) {
            // 500ms have passed or voltage is another volt higher, time for some tiltback
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->config.warnings.hv.tiltback_angle;
            } else {
                d->setpoint_target = -d->config.warnings.hv.tiltback_angle;
            }

            d->state.sat = SAT_PB_HIGH_VOLTAGE;
        } else {
            // The rider has 500ms to react to the triple-beep, or maybe it was just a short spike
            d->state.sat = SAT_NONE;
        }
    } else if (VESC_IF->mc_temp_fet_filtered() > d->mc_max_temp_fet) {
        // Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
        beep_alert(d, 3, true);
        d->beep_reason = BEEP_TEMPFET;
        if (VESC_IF->mc_temp_fet_filtered() > (d->mc_max_temp_fet + 1)) {
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->config.warnings.lv.tiltback_angle;
            } else {
                d->setpoint_target = -d->config.warnings.lv.tiltback_angle;
            }
            d->state.sat = SAT_PB_TEMPERATURE;
        } else {
            // The rider has 1 degree Celsius left before we start tilting back
            d->state.sat = SAT_NONE;
        }
    } else if (VESC_IF->mc_temp_motor_filtered() > d->mc_max_temp_mot) {
        // Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
        beep_alert(d, 3, true);
        d->beep_reason = BEEP_TEMPMOT;
        if (VESC_IF->mc_temp_motor_filtered() > (d->mc_max_temp_mot + 1)) {
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->config.warnings.lv.tiltback_angle;
            } else {
                d->setpoint_target = -d->config.warnings.lv.tiltback_angle;
            }
            d->state.sat = SAT_PB_TEMPERATURE;
        } else {
            // The rider has 1 degree Celsius left before we start tilting back
            d->state.sat = SAT_NONE;
        }
    } else if (d->motor.duty_cycle > 0.05 && input_voltage < d->config.warnings.lv.threshold) {
        beep_alert(d, 3, false);
        d->beep_reason = BEEP_LV;
        float abs_motor_current = fabsf(d->motor.current);
        float vdelta = d->config.warnings.lv.threshold - input_voltage;
        float ratio = vdelta * 20 / abs_motor_current;
        // When to do LV tiltback:
        // a) we're 2V below lv threshold
        // b) motor current is small (we cannot assume vsag)
        // c) we have more than 20A per Volt of difference (we tolerate some amount of vsag)
        if ((vdelta > 2) || (abs_motor_current < 5) || (ratio > 1)) {
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->config.warnings.lv.tiltback_angle;
            } else {
                d->setpoint_target = -d->config.warnings.lv.tiltback_angle;
            }

            d->state.sat = SAT_PB_LOW_VOLTAGE;
        } else {
            d->state.sat = SAT_NONE;
            d->setpoint_target = 0;
        }
    } else if (d->state.sat != SAT_CENTERING || d->setpoint_target_interpolated == d->setpoint_target) {
        // Normal running
        if (d->config.faults.is_reversestop_enabled && d->motor.erpm < -200) {
            d->state.sat = SAT_REVERSESTOP;
            d->reverse_timer = d->current_time;
            d->reverse_total_erpm = 0;
        } else {
            d->state.sat = SAT_NONE;
        }
        d->setpoint_target = 0;
    }

    if (d->state.wheelslip && d->motor.duty_cycle > d->max_duty_with_margin) {
        d->setpoint_target = 0;
    }

    if (d->state.sat == SAT_PB_DUTY) {
        if (d->config.warnings.is_dutybeep_enabled || (d->config.warnings.duty.tiltback_angle == 0)) {
            beep_on(d, true);
            d->beep_reason = BEEP_DUTY;
            d->duty_beeping = true;
        }
    } else {
        if (d->duty_beeping) {
            beep_off(d, false);
        }
    }
}

static void calculate_setpoint_interpolated(data *d) {
    if (d->setpoint_target_interpolated != d->setpoint_target) {
        rate_limitf(
            &d->setpoint_target_interpolated,
            d->setpoint_target,
            get_setpoint_adjustment_step_size(d)
        );
    }
}

void aggregate_tiltbacks(data *d) {
    d->setpoint += d->atr.interpolated;
    d->setpoint += d->torque_tilt.interpolated;
    d->setpoint += d->turn_tilt.interpolated;
    d->setpoint += d->speed_tilt.interpolated;
    d->setpoint += d->input_tilt.interpolated;
}

static void add_surge(data *d) {
    if (d->surge_enable) {
        float surge_now = 0;

        if (d->motor.duty_smooth > d->config.surge_duty_start + 0.04) {
            surge_now = d->surge_angle3;
            beep_alert(d, 3, 1);
        } else if (d->motor.duty_smooth > d->config.surge_duty_start + 0.02) {
            surge_now = d->surge_angle2;
            beep_alert(d, 2, 1);
        } else if (d->motor.duty_smooth > d->config.surge_duty_start) {
            surge_now = d->surge_angle;
            beep_alert(d, 1, 1);
        }
        if (surge_now >= d->surge_adder) {
            // kick in instantly
            d->surge_adder = surge_now;
        } else {
            // release less harshly
            d->surge_adder = d->surge_adder * 0.98 + surge_now * 0.02;
        }

        // Add surge angle to setpoint
        if (d->motor.erpm > 0) {
            d->setpoint += d->surge_adder;
        } else {
            d->setpoint -= d->surge_adder;
        }
    }
}

static void brake(data *d) {
    // Brake timeout logic
    float brake_timeout_length = 1;  // Brake Timeout hard-coded to 1s
    if (d->motor.erpm_abs > 1 || d->brake_timeout == 0) {
        d->brake_timeout = d->current_time + brake_timeout_length;
    }

    if (d->brake_timeout != 0 && d->current_time > d->brake_timeout) {
        return;
    }

    VESC_IF->timeout_reset();
    VESC_IF->mc_set_brake_current(d->config.brake_current);
}

static void set_current(float current) {
    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(0.025f);
    VESC_IF->mc_set_current(current);
}

static void imu_ref_callback(float *acc, float *gyro, [[maybe_unused]] float *mag, float dt) {
    data *d = (data *) ARG;
    balance_filter_update(&d->balance_filter, gyro, acc, dt);
}

static void time_vars_update(data *d) {
    d->current_time = VESC_IF->system_time();
    d->dt_raw = d->current_time - d->last_time;
    d->last_time = d->current_time;
    filter_ema(&d->dt, d->dt_raw, 0.1f);
}

static void refloat_thd(void *arg) {
    data *d = (data *) arg;

    configure(d);
    d->last_time = VESC_IF->system_time() - d->loop_time;

    while (!VESC_IF->should_terminate()) {
        time_vars_update(d);
        
        beeper_update(d);

        charging_timeout(&d->charging, &d->state);

        imu_data_update(&d->imu, &d->balance_filter);
        motor_data_update(&d->motor, d->config.hardware.esc.frequency);
        remote_data_update(&d->remote, &d->config.hardware.remote);
        footpad_sensor_update(&d->footpad_sensor, &d->config.faults);

        if (d->footpad_sensor.state == FS_NONE && d->state.state == STATE_RUNNING &&
            d->motor.erpm_abs > d->switch_warn_beep_erpm) {
            // If we're at riding speed and the switch is off => ALERT the user
            // set force=true since this could indicate an imminent shutdown/nosedive
            beep_on(d, true);
            d->beep_reason = BEEP_SENSORS;
        } else {
            // if the switch comes back on we stop beeping
            beep_off(d, false);
        }

        // Control Loop State Logic
        switch (d->state.state) {
        case (STATE_STARTUP):
            // Disable output
            brake(d);
            if (VESC_IF->imu_startup_done()) {
                reset_vars(d);
                // set state to READY so we need to meet start conditions to start
                d->state.state = STATE_READY;

                // if within 5V of LV tiltback threshold, issue 1 beep for each volt below that
                float bat_volts = VESC_IF->mc_get_input_voltage_filtered();
                float threshold = d->config.warnings.lv.threshold + 5;
                if (bat_volts < threshold) {
                    int beeps = (int) fminf(6, threshold - bat_volts);
                    beep_alert(d, beeps + 1, true);
                    d->beep_reason = BEEP_LOWBATT;
                } else {
                    // Let the rider know that the board is ready (one long beep)
                    beep_alert(d, 1, true);
                }
            }
            break;

        case (STATE_RUNNING):
            // Check for faults
            if (check_faults(d)) {
                if (d->state.stop_condition == STOP_SWITCH_FULL) {
                    // dirty landings: add extra margin when rightside up
                    d->startup_pitch_tolerance =
                        d->config.startup_pitch_tolerance + d->startup_pitch_trickmargin;
                    d->fault_angle_pitch_timer = d->current_time;
                }
                break;
            }
            d->odometer_dirty = 1;

            d->disengage_timer = d->current_time;

            // Calculate setpoint and interpolation
            calculate_setpoint_target(d);
            calculate_setpoint_interpolated(d);
            d->setpoint = d->setpoint_target_interpolated;

            add_surge(d);
            input_tilt_update(&d->input_tilt, &d->remote, &d->config.tune.input_tilt, d->loop_time);
            if (d->state.wheelslip) {
                atr_winddown(&d->atr);
                torque_tilt_winddown(&d->torque_tilt);
                turn_tilt_winddown(&d->turn_tilt);
                speed_tilt_winddown(&d->speed_tilt);
            } else {
                atr_update(&d->atr, &d->motor, &d->config.tune.atr, d->loop_time);
                torque_tilt_update(
                    &d->torque_tilt,
                    &d->motor,
                    &d->imu,
                    &d->config.tune.torque_tilt,
                    &d->config.tune.atr,
                    d->loop_time
                );
                turn_tilt_update(
                    &d->turn_tilt,
                    &d->motor,
                    &d->imu,
                    &d->atr,
                    &d->config.tune.turn_tilt,
                    d->loop_time
                );
                speed_tilt_update(
                    &d->speed_tilt, &d->motor, &d->config.tune.speed_tilt, d->loop_time
                );
            }

            aggregate_tiltbacks(d);

            pid_update(&d->pid, &d->imu, &d->motor, &d->config.tune.pid, d->setpoint);
            set_current(d->pid.pid_value);
            break;

        case (STATE_READY):
            // alert user after 30 minutes
            if (d->current_time - d->disengage_timer > 1800) {
                // beep every 60 seconds
                if (d->current_time - d->nag_timer > 60) {
                    d->nag_timer = d->current_time;
                    float input_voltage = VESC_IF->mc_get_input_voltage_filtered();
                    if (input_voltage > d->idle_voltage) {
                        // don't beep if the voltage keeps increasing (board is charging)
                        d->idle_voltage = input_voltage;
                    } else {
                        d->beep_reason = BEEP_IDLE;
                        beep_alert(d, 2, 1);
                    }
                }
            } else {
                d->nag_timer = d->current_time;
                d->idle_voltage = 0;
            }

            if ((d->current_time - d->fault_angle_pitch_timer) > 1) {
                // 1 second after disengaging - set startup tolerance back to normal (aka tighter)
                d->startup_pitch_tolerance = d->config.startup_pitch_tolerance;
            }

            check_odometer(d);

            // Check for valid startup position and switch state
            if (is_orientation_valid(d) && is_sensor_engaged(d)) {
                reset_vars(d);
                break;
            }
            // Push-start aka dirty landing Part II
            if (d->config.startup_pushstart_enabled && d->motor.erpm_abs > 1000 &&
                is_sensor_engaged(d)) {
                if ((fabsf(d->imu.pitch_balance) < 45) && (fabsf(d->imu.roll) < 45)) {
                    // 45 to prevent board engaging when upright or laying sideways
                    // 45 degree tolerance is more than plenty for tricks / extreme mounts
                    reset_vars(d);
                    break;
                }
            }

            // Set RC current or maintain brake current (and keep WDT happy!)
            do_rc_move(d);
            break;
        case (STATE_DISABLED):
            // no set_current, no brake_current
            break;
        }

        d->time_diff = VESC_IF->system_time() - d->current_time;
        const uint32_t time_sleep_us = max(d->loop_time_us - 1e6 * d->time_diff, 0);
        VESC_IF->sleep_us(time_sleep_us);
    }
}

static void write_cfg_to_eeprom(data *d) {
    uint32_t ints = sizeof(RefloatConfig) / 4 + 1;
    uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
    if (!buffer) {
        log_error("Failed to write config to EEPROM: Out of memory.");
        return;
    }

    bool write_ok = true;
    memcpy(buffer, &(d->config), sizeof(RefloatConfig));
    for (uint32_t i = 0; i < ints; i++) {
        eeprom_var v;
        v.as_u32 = buffer[i];
        if (!VESC_IF->store_eeprom_var(&v, i + 1)) {
            write_ok = false;
            break;
        }
    }

    VESC_IF->free(buffer);

    if (write_ok) {
        eeprom_var v;
        v.as_u32 = REFLOATCONFIG_SIGNATURE;
        VESC_IF->store_eeprom_var(&v, 0);
    } else {
        log_error("Failed to write config to EEPROM.");
    }

    beep_alert(d, 1, 0);
}

static void led_thd(void *arg) {
    data *d = (data *) arg;

    while (!VESC_IF->should_terminate()) {
        leds_update(&d->leds, &d->state, d->footpad_sensor.state);
        VESC_IF->sleep_us(1e6 / LEDS_REFRESH_RATE);
    }
}

static void read_cfg_from_eeprom(RefloatConfig *config) {
    uint32_t ints = sizeof(RefloatConfig) / 4 + 1;
    uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
    if (!buffer) {
        log_error("Failed to read config from EEPROM: Out of memory.");
        return;
    }

    eeprom_var v;
    bool read_ok = VESC_IF->read_eeprom_var(&v, 0);
    if (read_ok) {
        if (v.as_u32 == REFLOATCONFIG_SIGNATURE) {
            for (uint32_t i = 0; i < ints; i++) {
                if (!VESC_IF->read_eeprom_var(&v, i + 1)) {
                    read_ok = false;
                    break;
                }
                buffer[i] = v.as_u32;
            }
        } else {
            log_error("Failed signature check while reading config from EEPROM, using defaults.");
            confparser_set_defaults_refloatconfig(config);
            return;
        }
    }

    if (read_ok) {
        memcpy(config, buffer, sizeof(RefloatConfig));
    } else {
        confparser_set_defaults_refloatconfig(config);
        log_error("Failed to read config from EEPROM, using defaults.");
    }

    VESC_IF->free(buffer);
}

static void data_init(data *d) {
    memset(d, 0, sizeof(data));

    read_cfg_from_eeprom(&d->config);

    d->odometer = VESC_IF->mc_get_odometer();

    lcm_init(&d->lcm, &d->config.hardware.leds);
    charging_init(&d->charging);
}

static float app_get_debug(int index) {
    data *d = (data *) ARG;

    switch (index) {
    case (1):
        return d->pid.pid_value;
    case (2):
        return d->pid.proportional;
    case (3):
        return d->pid.integral;
    case (4):
        return d->pid.derivative;
    case (5):
        return d->setpoint;
    case (6):
        return d->atr.interpolated;
    case (7):
        return d->motor.erpm;
    case (8):
        return d->motor.current;
    case (9):
        return d->motor.current_filtered;
    default:
        return 0;
    }
}

// See also:
// LcmCommands in lcm.h
// ChargingCommands in charging.h
enum {
    COMMAND_GET_INFO = 0,  // get version / package info
    COMMAND_GET_RTDATA = 1,  // get rt data
    // COMMAND_RT_TUNE = 2,  // runtime tuning (don't write to eeprom)
    // COMMAND_TUNE_DEFAULTS = 3,  // set tune to defaults (no eeprom)
    COMMAND_CFG_SAVE = 4,  // save config to eeprom
    COMMAND_CFG_RESTORE = 5,  // restore config from eeprom
    // COMMAND_TUNE_OTHER = 6,  // make runtime changes to startup/etc
    COMMAND_RC_MOVE = 7,  // move motor while board is idle
    // COMMAND_BOOSTER = 8,  // change booster settings
    COMMAND_PRINT_INFO = 9,  // print verbose info
    // COMMAND_GET_ALLDATA = 10,  // send all data, compact
    // COMMAND_EXPERIMENT = 11,  // generic cmd for sending data, used for testing/tuning new features
    COMMAND_LOCK = 12,
    COMMAND_HANDTEST = 13,
    // COMMAND_TUNE_TILT = 14,

    // commands above 200 are unstable and can change protocol at any time
    COMMAND_GET_RTDATA_2 = 201,
    COMMAND_LIGHTS_CONTROL = 202,
} Commands;

static void cmd_print_info([[maybe_unused]] data *d) {
}

static void cmd_lock(data *d, unsigned char *cfg) {
    if (d->state.state < STATE_RUNNING) {
        d->config.disabled = cfg[0] ? true : false;
        d->state.state = cfg[0] ? STATE_DISABLED : STATE_STARTUP;
        write_cfg_to_eeprom(d);
    }
}

static void cmd_handtest(data *d, unsigned char *cfg) {
    if (d->state.state != STATE_READY) {
        return;
    }

    if (d->state.mode != MODE_NORMAL && d->state.mode != MODE_HANDTEST) {
        return;
    }

    d->state.mode = cfg[0] ? MODE_HANDTEST : MODE_NORMAL;
    if (d->state.mode == MODE_HANDTEST) {
        d->motor.current_max = d->motor.current_min = 7;
        d->config.tune.pid.ki = 0.0f;
        d->config.tune.pid.kp_brake = 1.0f;
        d->config.tune.pid.kd_brake = 1.0f;
        d->config.tune.torque_tilt.strength = 0.0f;
        d->config.tune.torque_tilt.strength_regen = 0.0f;
        d->config.tune.atr.strength_up = 0.0f;
        d->config.tune.atr.strength_down = 0.0f;
        d->config.tune.turn_tilt.strength = 0.0f;
        d->config.tune.speed_tilt.constant = 0.0f;
        d->config.tune.speed_tilt.variable = 0.0f;
        d->config.faults.pitch_threshold = 30;
        d->config.faults.roll_threshold = 30;
    } else {
        read_cfg_from_eeprom(&d->config);
        configure(d);
    }
}

void cmd_rc_move(data *d, unsigned char *cfg) {
    int ind = 0;
    int direction = cfg[ind++];
    int current = cfg[ind++];
    int time = cfg[ind++];
    int sum = cfg[ind++];
    if (sum != time + current) {
        current = 0;
    } else if (direction == 0) {
        current = -current;
    }

    if (d->state.state == STATE_READY) {
        d->rc_counter = 0;
        if (current == 0) {
            d->rc_steps = 1;
            d->rc_current_target = 0;
            d->rc_current = 0;
        } else {
            d->rc_steps = time * 100;
            d->rc_current_target = current / 10.0;
            if (d->rc_current_target > 8) {
                d->rc_current_target = 2;
            }
        }
    }
}

static void send_realtime_data(data *d) {
    static const int bufsize = 67;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_GET_RTDATA_2;

    // mask indicates what groups of data are sent, to prevent sending data
    // that are not useful in a given state
    uint8_t mask = 0;
    if (d->state.state == STATE_RUNNING) {
        mask |= 0x1;
    }

    if (d->state.charging) {
        mask |= 0x2;
    }

    buffer[ind++] = mask;

    buffer[ind++] = d->state.mode << 4 | d->state.state;

    uint8_t flags = d->state.charging << 5 | false << 1 | d->state.wheelslip;
    buffer[ind++] = d->footpad_sensor.state << 6 | flags;

    buffer[ind++] = d->state.sat << 4 | d->state.stop_condition;

    buffer[ind++] = d->beep_reason;

    buffer_append_float32_auto(buffer, d->imu.pitch, &ind);
    buffer_append_float32_auto(buffer, d->imu.pitch_balance, &ind);
    buffer_append_float32_auto(buffer, d->imu.roll, &ind);

    buffer_append_float32_auto(buffer, d->footpad_sensor.adc1, &ind);
    buffer_append_float32_auto(buffer, d->footpad_sensor.adc2, &ind);
    buffer_append_float32_auto(buffer, d->remote.throttle, &ind);

    if (d->state.state == STATE_RUNNING) {
        // Setpoints
        buffer_append_float32_auto(buffer, d->setpoint, &ind);

        buffer_append_float32_auto(buffer, d->atr.interpolated, &ind);
        buffer_append_float32_auto(buffer, d->torque_tilt.interpolated, &ind);
        buffer_append_float32_auto(buffer, d->turn_tilt.interpolated, &ind);
        buffer_append_float32_auto(buffer, d->speed_tilt.interpolated, &ind);
        buffer_append_float32_auto(buffer, d->input_tilt.interpolated, &ind);

        // DEBUG
        buffer_append_float32_auto(buffer, d->pid.pid_value, &ind);
        buffer_append_float32_auto(buffer, d->motor.current_filtered, &ind);
        buffer_append_float32_auto(buffer, d->config.tune.atr.strength_up, &ind);
    }

    if (d->state.charging) {
        buffer_append_float32_auto(buffer, d->charging.current, &ind);
        buffer_append_float32_auto(buffer, d->charging.voltage, &ind);
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

static void lights_control_request(CfgLeds *leds, uint8_t *buffer, size_t len, LcmData *lcm) {
    if (len < 2) {
        return;
    }

    uint8_t mask = buffer[0];
    uint8_t value = buffer[1];

    if (mask != 0) {
        if (mask & 0x1) {
            leds->on = value & 0x1;
        }

        if (mask & 0x2) {
            leds->headlights_on = value & 0x2;
        }

        if (lcm->enabled) {
            lcm_configure(lcm, leds);
        }
    }
}

static void lights_control_response(const CfgLeds *leds) {
    static const int bufsize = 3;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LIGHTS_CONTROL;
    buffer[ind++] = leds->headlights_on << 1 | leds->on;

    SEND_APP_DATA(buffer, bufsize, ind);
}

// Handler for incoming app commands
static void on_command_received(unsigned char *buffer, unsigned int len) {
    data *d = (data *) ARG;
    uint8_t magicnr = buffer[0];
    uint8_t command = buffer[1];

    if (len < 2) {
        log_error("Received command data too short.");
        return;
    }
    if (magicnr != 101) {
        log_error("Invalid Package ID: %u", magicnr);
        return;
    }

    switch (command) {
        case COMMAND_GET_INFO: {
            int32_t ind = 0;
            uint8_t send_buffer[10];
            send_buffer[ind++] = 101;  // magic nr.
            send_buffer[ind++] = 0x0;  // command ID
            send_buffer[ind++] = (uint8_t) (10 * PACKAGE_MAJOR_MINOR_VERSION);
            send_buffer[ind++] = 1;  // build number
            // Send the full type here. This is redundant with cmd_light_info. It
            // likely shouldn't be here, as the type can be reconfigured and the
            // app would need to reconnect to pick up the change from this command.
            send_buffer[ind++] = d->config.hardware.leds.type;
            VESC_IF->send_app_data(send_buffer, ind);
            return;
        }
        case COMMAND_GET_RTDATA_2: {
            send_realtime_data(d);
            return;
        }
        // case COMMAND_TUNE_OTHER: {
        //     if (len >= 14) {
        //         cmd_runtime_tune_other(d, &buffer[2], len - 2);
        //     } else {
        //         log_error("Command data length incorrect: %u", len);
        //     }
        //     return;
        // }
        // case COMMAND_TUNE_TILT: {
        //     if (len >= 10) {
        //         cmd_runtime_tune_tilt(d, &buffer[2], len - 2);
        //     } else {
        //         log_error("Command data length incorrect: %u", len);
        //     }
        //     return;
        // }
        case COMMAND_RC_MOVE: {
            if (len == 6) {
                cmd_rc_move(d, &buffer[2]);
            } else {
                log_error("Command data length incorrect: %u", len);
            }
            return;
        }
        case COMMAND_CFG_RESTORE: {
            read_cfg_from_eeprom(&d->config);
            return;
        }
        case COMMAND_CFG_SAVE: {
            write_cfg_to_eeprom(d);
            return;
        }
        // case COMMAND_TUNE_DEFAULTS: {
        //     cmd_tune_defaults(d);
        //     return;
        // }
        case COMMAND_PRINT_INFO: {
            cmd_print_info(d);
            return;
        }
        // case COMMAND_GET_ALLDATA: {
        //     if (len == 3) {
        //         cmd_send_all_data(d, buffer[2]);
        //     } else {
        //         log_error("Command data length incorrect: %u", len);
        //     }
        //     return;
        // }
        // case COMMAND_EXPERIMENT: {
        //     cmd_experiment(d, &buffer[2]);
        //     return;
        // }
        case COMMAND_LOCK: {
            cmd_lock(d, &buffer[2]);
            return;
        }
        case COMMAND_HANDTEST: {
            cmd_handtest(d, &buffer[2]);
            return;
        }
        case COMMAND_LCM_POLL: {
            lcm_poll_request(&d->lcm, &buffer[2], len - 2);
            lcm_poll_response(&d->lcm, &d->state, d->footpad_sensor.state, &d->motor, d->imu.pitch);
            return;
        }
        case COMMAND_LCM_LIGHT_INFO: {
            lcm_light_info_response(&d->lcm);
            return;
        }
        case COMMAND_LCM_LIGHT_CTRL: {
            lcm_light_ctrl_request(&d->lcm, &buffer[2], len - 2);
            return;
        }
        case COMMAND_LCM_DEVICE_INFO: {
            lcm_device_info_response(&d->lcm);
            return;
        }
        case COMMAND_LCM_GET_BATTERY: {
            lcm_get_battery_response(&d->lcm);
            return;
        }
        case COMMAND_CHARGING_STATE: {
            charging_state_request(&d->charging, &buffer[2], len - 2, &d->state);
            return;
        }
        case COMMAND_LIGHTS_CONTROL: {
            lights_control_request(&d->config.leds, &buffer[2], len - 2, &d->lcm);
            lights_control_response(&d->config.leds);
            return;
        }
        default: {
            if (!VESC_IF->app_is_output_disabled()) {
                log_error("Unknown command received: %u", command);
            }
        }
    }
}

// Register get_debug as a lisp extension
static lbm_value ext_dbg(lbm_value *args, lbm_uint argn) {
    if (argn != 1 || !VESC_IF->lbm_is_number(args[0])) {
        return VESC_IF->lbm_enc_sym_eerror;
    }

    return VESC_IF->lbm_enc_float(app_get_debug(VESC_IF->lbm_dec_as_i32(args[0])));
}

// Called from Lisp on init to pass in the version info of the firmware
static lbm_value ext_set_fw_version(lbm_value *args, lbm_uint argn) {
    data *d = (data *) ARG;
    if (argn > 2) {
        d->fw_version_major = VESC_IF->lbm_dec_as_i32(args[0]);
        d->fw_version_minor = VESC_IF->lbm_dec_as_i32(args[1]);
        d->fw_version_beta = VESC_IF->lbm_dec_as_i32(args[2]);
    }
    return VESC_IF->lbm_enc_sym_true;
}

// Used to send the current or default configuration to VESC Tool.
static int get_cfg(uint8_t *buffer, bool is_default) {
    data *d = (data *) ARG;

    RefloatConfig *cfg;
    if (is_default) {
        cfg = VESC_IF->malloc(sizeof(RefloatConfig));
        if (!cfg) {
            log_error("Failed to send default config to VESC tool: Out of memory.");
            return 0;
        }
        confparser_set_defaults_refloatconfig(cfg);
    } else {
        cfg = &d->config;
    }

    int res = confparser_serialize_refloatconfig(buffer, cfg);

    if (is_default) {
        VESC_IF->free(cfg);
    }

    return res;
}

// Used to set and write configuration from VESC Tool.
static bool set_cfg(uint8_t *buffer) {
    data *d = (data *) ARG;

    // don't let users use the Refloat Cfg "write" button in special modes
    if (d->state.mode != MODE_NORMAL) {
        return false;
    }

    bool res = confparser_deserialize_refloatconfig(buffer, &d->config);

    // Store to EEPROM
    if (res) {
        write_cfg_to_eeprom(d);
        configure(d);
        leds_configure(&d->leds, &d->config.leds);
    }

    return res;
}

static int get_cfg_xml(uint8_t **buffer) {
    // Note: As the address of data_refloatconfig_ is not known
    // at compile time it will be relative to where it is in the
    // linked binary. Therefore we add PROG_ADDR to it so that it
    // points to where it ends up on the STM32.
    *buffer = data_refloatconfig_ + PROG_ADDR;
    return DATA_REFLOATCONFIG__SIZE;
}

// Called when code is stopped
static void stop(void *arg) {
    data *d = (data *) arg;
    VESC_IF->imu_set_read_callback(NULL);
    VESC_IF->set_app_data_handler(NULL);
    VESC_IF->conf_custom_clear_configs();
    if (d->led_thread) {
        VESC_IF->request_terminate(d->led_thread);
    }
    if (d->main_thread) {
        VESC_IF->request_terminate(d->main_thread);
    }
    log_msg("Terminating.");
    leds_destroy(&d->leds);
    VESC_IF->free(d);
}

INIT_FUN(lib_info *info) {
    INIT_START
    log_msg("Initializing Refloat v" PACKAGE_VERSION);

    data *d = VESC_IF->malloc(sizeof(data));
    if (!d) {
        log_error("Out of memory, startup failed.");
        return false;
    }
    data_init(d);

    info->stop_fun = stop;
    info->arg = d;

    VESC_IF->conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml);

    if ((d->config.hardware.esc.is_beeper_enabled) ||
        (d->config.hardware.remote.type != INPUTTILT_PPM)) {
        beeper_init();
    }

    balance_filter_init(&d->balance_filter);
    VESC_IF->imu_set_read_callback(imu_ref_callback);

    footpad_sensor_update(&d->footpad_sensor, &d->config.faults);

    d->main_thread = VESC_IF->spawn(refloat_thd, 1024, "Refloat Main", d);
    if (!d->main_thread) {
        log_error("Failed to spawn Refloat Main thread.");
        return false;
    }

    bool have_leds = leds_init(
        &d->leds, &d->config.hardware.leds, &d->config.leds, d->footpad_sensor.state
    );

    if (have_leds) {
        d->led_thread = VESC_IF->spawn(led_thd, 1024, "Refloat LEDs", d);
        if (!d->led_thread) {
            log_error("Failed to spawn Refloat LEDs thread.");
            leds_destroy(&d->leds);
        }
    }

    VESC_IF->set_app_data_handler(on_command_received);
    VESC_IF->lbm_add_extension("ext-dbg", ext_dbg);
    VESC_IF->lbm_add_extension("ext-set-fw-version", ext_set_fw_version);

    return true;
}

void send_app_data_overflow_terminate() {
    VESC_IF->request_terminate(((data *) ARG)->main_thread);
}
