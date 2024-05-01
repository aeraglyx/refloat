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

#include "atr.h"
#include "torque_tilt.h"
#include "turn_tilt.h"
#include "speed_tilt.h"

#include "pid.h"
#include "balance_filter.h"
#include "charging.h"
#include "footpad_sensor.h"
#include "lcm.h"
#include "leds.h"
#include "motor_data.h"
#include "imu_data.h"
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

    RefloatConfig float_conf;

    // Firmware version, passed in from Lisp
    int fw_version_major, fw_version_minor, fw_version_beta;

    State state;

    // Board data
    IMUData imu;
    MotorData motor;

    // IMU data for the balancing filter
    BalanceFilterData balance_filter;

    // Tune modifiers
    ATR atr;
    TorqueTilt torque_tilt;
    TurnTilt turn_tilt;
    SpeedTilt speed_tilt;

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
    float loop_time_s;
    uint32_t loop_time_us;

    float startup_pitch_trickmargin, startup_pitch_tolerance;
    float startup_step_size;
    float tiltback_duty_step_size, tiltback_hv_step_size, tiltback_lv_step_size,
        tiltback_return_step_size;
    float inputtilt_ramped_step_size, inputtilt_step_size;
    float mc_max_temp_fet, mc_max_temp_mot;
    float mc_current_max, mc_current_min;
    float surge_angle, surge_angle2, surge_angle3, surge_adder;
    bool surge_enable;
    bool duty_beeping;

    float throttle_val;
    float max_duty_with_margin;

    FootpadSensor footpad_sensor;

    PID pid;

    float setpoint, setpoint_target, setpoint_target_interpolated;
    float inputtilt_interpolated;
    float current_time;
    float disengage_timer, nag_timer;
    float idle_voltage;
    float fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer,
        fault_switch_half_timer;
    float motor_timeout_s;
    float brake_timeout;
    float wheelslip_timer, tb_highvoltage_timer;
    float switch_warn_beep_erpm;

    // Feature: Reverse Stop
    float reverse_stop_step_size, reverse_tolerance, reverse_total_erpm;
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
static void set_current(data *d, float current);

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

static void reconfigure(data *d) {
    motor_data_configure(&d->motor, &d->float_conf);
    balance_filter_configure(&d->balance_filter, &d->float_conf);

    atr_configure(&d->atr, &d->float_conf);
    torque_tilt_configure(&d->torque_tilt, &d->float_conf);
    turn_tilt_configure(&d->turn_tilt, &d->float_conf);
    speed_tilt_configure(&d->speed_tilt, &d->float_conf);

    pid_configure(&d->pid, &d->float_conf);
}

static void configure(data *d) {
    state_init(&d->state, d->float_conf.disabled);

    lcm_configure(&d->lcm, &d->float_conf.leds);

    // This timer is used to determine how long the board has been disengaged / idle
    d->disengage_timer = d->current_time;

    // Loop time in seconds and microseconds
    d->loop_time_s = 1.0f / d->float_conf.hertz;
    d->loop_time_us = 1e6 / d->float_conf.hertz;

    // Loop time in seconds times 20 for a nice long grace period
    d->motor_timeout_s = 20.0f / d->float_conf.hertz;

    d->startup_step_size = d->float_conf.startup_speed / d->float_conf.hertz;
    d->inputtilt_step_size = d->float_conf.inputtilt_speed_max / d->float_conf.hertz;

    d->tiltback_duty_step_size = d->float_conf.tiltback_duty_speed / d->float_conf.hertz;
    d->tiltback_hv_step_size = d->float_conf.tiltback_hv_speed / d->float_conf.hertz;
    d->tiltback_lv_step_size = d->float_conf.tiltback_lv_speed / d->float_conf.hertz;
    d->tiltback_return_step_size = d->float_conf.tiltback_return_speed / d->float_conf.hertz;

    d->surge_angle = d->float_conf.surge_angle;
    d->surge_angle2 = d->float_conf.surge_angle * 2;
    d->surge_angle3 = d->float_conf.surge_angle * 3;
    d->surge_enable = d->surge_angle > 0;

    // Feature: Dirty Landings
    d->startup_pitch_trickmargin = d->float_conf.startup_dirtylandings_enabled ? 10 : 0;

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

    d->mc_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    // min current is a positive value here!
    d->mc_current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));

    d->max_duty_with_margin = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty) - 0.1;

    // Feature: Reverse Stop
    d->reverse_tolerance = 50000;
    d->reverse_stop_step_size = 100.0 / d->float_conf.hertz;

    // Allows smoothing of Remote Tilt
    d->inputtilt_ramped_step_size = 0;

    // Speed above which to warn users about an impending full switch fault
    d->switch_warn_beep_erpm = d->float_conf.is_footbeep_enabled ? 2000 : 100000;

    d->beeper_enabled = d->float_conf.is_beeper_enabled;

    reconfigure(d);

    if (d->state.state == STATE_DISABLED) {
        beep_alert(d, 3, false);
    } else {
        beep_alert(d, 1, false);
    }
}

static void reset_vars(data *d) {
    imu_data_reset(&d->imu);
    motor_data_reset(&d->motor);

    atr_reset(&d->atr);
    torque_tilt_reset(&d->torque_tilt);
    turn_tilt_reset(&d->turn_tilt);
    speed_tilt_reset(&d->speed_tilt);

    pid_reset(&d->pid);

    // Set values for startup
    d->setpoint = d->imu.pitch_balance;
    d->setpoint_target_interpolated = d->imu.pitch_balance;
    d->setpoint_target = 0;
    d->inputtilt_interpolated = 0;
    d->brake_timeout = 0;

    d->startup_pitch_tolerance = d->float_conf.startup_pitch_tolerance;
    d->surge_adder = 0;

    // RC Move:
    d->rc_steps = 0;
    d->rc_current = 0;

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
        set_current(d, d->rc_current);
        d->rc_steps--;
        d->rc_counter++;
        if ((d->rc_counter == 500) && (d->rc_current_target > 2)) {
            d->rc_current_target /= 2;
        }
    } else {
        d->rc_counter = 0;

        // Throttle must be greater than 2% (Help mitigate lingering throttle)
        if ((d->float_conf.remote_throttle_current_max > 0) &&
            (d->current_time - d->disengage_timer > d->float_conf.remote_throttle_grace_period) &&
            (fabsf(d->throttle_val) > 0.02)) {
            float servo_val = d->throttle_val;
            servo_val *= (d->float_conf.inputtilt_invert_throttle ? -1.0 : 1.0);
            d->rc_current = d->rc_current * 0.95 +
                (d->float_conf.remote_throttle_current_max * servo_val) * 0.05;
            set_current(d, d->rc_current);
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

bool is_engaged(const data *d) {
    if (d->footpad_sensor.state == FS_BOTH) {
        return true;
    }

    if (d->footpad_sensor.state == FS_LEFT || d->footpad_sensor.state == FS_RIGHT) {
        // 5 seconds after stopping we allow starting with a single sensor (e.g. for jump starts)
        bool is_simple_start =
            d->float_conf.startup_simplestart_enabled && (d->current_time - d->disengage_timer > 5);

        if (d->float_conf.fault_is_dual_switch || is_simple_start) {
            return true;
        }
    }

    return false;
}

// Fault checking order does not really matter. From a UX perspective, switch should be before
// angle.
static bool check_faults(data *d) {
    bool disable_switch_faults = d->float_conf.fault_moving_fault_disabled &&
        // Rolling forward (not backwards!)
        d->motor.erpm > (d->float_conf.fault_adc_half_erpm * 2) &&
        // Not tipped over
        fabsf(d->imu.roll) < 40;

    // Check switch
    // Switch fully open
    if (d->footpad_sensor.state == FS_NONE) {
        if (!disable_switch_faults) {
            if ((1000.0 * (d->current_time - d->fault_switch_timer)) >
                d->float_conf.fault_delay_switch_full) {
                state_stop(&d->state, STOP_SWITCH_FULL);
                return true;
            }
            // low speed (below 6 x half-fault threshold speed):
            else if (
                (d->motor.erpm_abs < d->float_conf.fault_adc_half_erpm * 6) &&
                (1000.0 * (d->current_time - d->fault_switch_timer) >
                    d->float_conf.fault_delay_switch_half)) {
                state_stop(&d->state, STOP_SWITCH_FULL);
                return true;
            }
        }

        if (d->motor.erpm_abs < 200 && fabsf(d->imu.pitch) > 14 &&
            fabsf(d->inputtilt_interpolated) < 30 && sign(d->imu.pitch) == d->motor.erpm_sign) {
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
        if (fabsf(d->imu.pitch) < 5) {
            d->reverse_timer = d->current_time;
        }
    }

    // Switch partially open and stopped
    if (!d->float_conf.fault_is_dual_switch) {
        if (!is_engaged(d) && d->motor.erpm_abs < d->float_conf.fault_adc_half_erpm) {
            if ((1000.0 * (d->current_time - d->fault_switch_half_timer)) >
                d->float_conf.fault_delay_switch_half) {
                state_stop(&d->state, STOP_SWITCH_HALF);
                return true;
            }
        } else {
            d->fault_switch_half_timer = d->current_time;
        }
    }

    // Check roll angle
    if (fabsf(d->imu.roll) > d->float_conf.fault_roll) {
        if ((1000.0 * (d->current_time - d->fault_angle_roll_timer)) >
            d->float_conf.fault_delay_roll) {
            state_stop(&d->state, STOP_ROLL);
            return true;
        }
    } else {
        d->fault_angle_roll_timer = d->current_time;
    }

    // Check pitch angle
    if (fabsf(d->imu.pitch) > d->float_conf.fault_pitch && fabsf(d->inputtilt_interpolated) < 30) {
        if ((1000.0 * (d->current_time - d->fault_angle_pitch_timer)) >
            d->float_conf.fault_delay_pitch) {
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

    if (input_voltage < d->float_conf.tiltback_hv) {
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
        fabsf(d->motor.acceleration) > 15 &&  // not normal, either wheelslip or wheel getting stuck
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
        if (d->float_conf.fault_reversestop_enabled && (d->motor.erpm < 0)) {
            // the 500ms wheelslip time can cause us to blow past the reverse stop condition!
            d->state.sat = SAT_REVERSESTOP;
            d->reverse_timer = d->current_time;
            d->reverse_total_erpm = 0;
        }
    } else if (d->motor.duty_cycle > d->float_conf.tiltback_duty) {
        if (d->motor.erpm > 0) {
            d->setpoint_target = d->float_conf.tiltback_duty_angle;
        } else {
            d->setpoint_target = -d->float_conf.tiltback_duty_angle;
        }
        d->state.sat = SAT_PB_DUTY;
    } else if (d->motor.duty_cycle > 0.05 && input_voltage > d->float_conf.tiltback_hv) {
        d->beep_reason = BEEP_HV;
        beep_alert(d, 3, false);
        if (((d->current_time - d->tb_highvoltage_timer) > .5) ||
            (input_voltage > d->float_conf.tiltback_hv + 1)) {
            // 500ms have passed or voltage is another volt higher, time for some tiltback
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->float_conf.tiltback_hv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_hv_angle;
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
                d->setpoint_target = d->float_conf.tiltback_lv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_lv_angle;
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
                d->setpoint_target = d->float_conf.tiltback_lv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_lv_angle;
            }
            d->state.sat = SAT_PB_TEMPERATURE;
        } else {
            // The rider has 1 degree Celsius left before we start tilting back
            d->state.sat = SAT_NONE;
        }
    } else if (d->motor.duty_cycle > 0.05 && input_voltage < d->float_conf.tiltback_lv) {
        beep_alert(d, 3, false);
        d->beep_reason = BEEP_LV;
        float abs_motor_current = fabsf(d->motor.current);
        float vdelta = d->float_conf.tiltback_lv - input_voltage;
        float ratio = vdelta * 20 / abs_motor_current;
        // When to do LV tiltback:
        // a) we're 2V below lv threshold
        // b) motor current is small (we cannot assume vsag)
        // c) we have more than 20A per Volt of difference (we tolerate some amount of vsag)
        if ((vdelta > 2) || (abs_motor_current < 5) || (ratio > 1)) {
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->float_conf.tiltback_lv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_lv_angle;
            }

            d->state.sat = SAT_PB_LOW_VOLTAGE;
        } else {
            d->state.sat = SAT_NONE;
            d->setpoint_target = 0;
        }
    } else if (d->state.sat != SAT_CENTERING || d->setpoint_target_interpolated == d->setpoint_target) {
        // Normal running
        if (d->float_conf.fault_reversestop_enabled && d->motor.erpm < -200) {
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
        if (d->float_conf.is_dutybeep_enabled || (d->float_conf.tiltback_duty_angle == 0)) {
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

void aggregate_tiltbacks(data *d){
    d->setpoint += d->atr.interpolated;
    // d->setpoint += d->atr.braketilt_interpolated;
    d->setpoint += d->torque_tilt.interpolated;
    d->setpoint += d->turn_tilt.interpolated;
    d->setpoint += d->speed_tilt.interpolated;
    // float ab_offset = d->atr.offset + d->atr.braketilt_offset;
    // if (sign(ab_offset) == sign(d->torque_tilt.offset)) {
    //     d->setpoint +=
    //         sign(ab_offset) * fmaxf(fabsf(ab_offset), fabsf(d->torque_tilt.offset));
    // } else {
    //     d->setpoint += ab_offset + d->torque_tilt.offset;
    // }
}

static void add_surge(data *d) {
    if (d->surge_enable) {
        float surge_now = 0;

        if (d->motor.duty_smooth > d->float_conf.surge_duty_start + 0.04) {
            surge_now = d->surge_angle3;
            beep_alert(d, 3, 1);
        } else if (d->motor.duty_smooth > d->float_conf.surge_duty_start + 0.02) {
            surge_now = d->surge_angle2;
            beep_alert(d, 2, 1);
        } else if (d->motor.duty_smooth > d->float_conf.surge_duty_start) {
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

static void apply_inputtilt(data *d) {
    float input_tiltback_target;

    // Scale by Max Angle
    input_tiltback_target = d->throttle_val * d->float_conf.inputtilt_angle_limit;

    float input_tiltback_target_diff = input_tiltback_target - d->inputtilt_interpolated;

    // Smoothen changes in tilt angle by ramping the step size
    if (d->float_conf.inputtilt_smoothing_factor > 0) {
        float smoothing_factor = 0.02;
        for (int i = 1; i < d->float_conf.inputtilt_smoothing_factor; i++) {
            smoothing_factor /= 2;
        }

        // Sets the angle away from Target that step size begins ramping down
        float smooth_center_window = 1.5 + (0.5 * d->float_conf.inputtilt_smoothing_factor);

        // Within X degrees of Target Angle, start ramping down step size
        if (fabsf(input_tiltback_target_diff) < smooth_center_window) {
            // Target step size is reduced the closer to center you are (needed for smoothly
            // transitioning away from center)
            d->inputtilt_ramped_step_size =
                (smoothing_factor * d->inputtilt_step_size * (input_tiltback_target_diff / 2)) +
                ((1 - smoothing_factor) * d->inputtilt_ramped_step_size);
            // Linearly ramped down step size is provided as minimum to prevent overshoot
            float centering_step_size =
                fminf(
                    fabsf(d->inputtilt_ramped_step_size),
                    fabsf(input_tiltback_target_diff / 2) * d->inputtilt_step_size
                ) *
                sign(input_tiltback_target_diff);
            if (fabsf(input_tiltback_target_diff) < fabsf(centering_step_size)) {
                d->inputtilt_interpolated = input_tiltback_target;
            } else {
                d->inputtilt_interpolated += centering_step_size;
            }
        } else {
            // Ramp up step size until the configured tilt speed is reached
            d->inputtilt_ramped_step_size =
                (smoothing_factor * d->inputtilt_step_size * sign(input_tiltback_target_diff)) +
                ((1 - smoothing_factor) * d->inputtilt_ramped_step_size);
            d->inputtilt_interpolated += d->inputtilt_ramped_step_size;
        }
    } else {
        // Constant step size; no smoothing
        if (fabsf(input_tiltback_target_diff) < d->inputtilt_step_size) {
            d->inputtilt_interpolated = input_tiltback_target;
        } else {
            d->inputtilt_interpolated += d->inputtilt_step_size * sign(input_tiltback_target_diff);
        }
    }

    d->setpoint += d->inputtilt_interpolated;
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
    VESC_IF->mc_set_brake_current(d->float_conf.brake_current);
}

static void set_current(data *d, float current) {
    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
    VESC_IF->mc_set_current(current);
}

static void imu_ref_callback(float *acc, float *gyro, [[maybe_unused]] float *mag, float dt) {
    data *d = (data *) ARG;
    balance_filter_update(&d->balance_filter, gyro, acc, dt);
}

static void refloat_thd(void *arg) {
    data *d = (data *) arg;

    configure(d);

    while (!VESC_IF->should_terminate()) {
        beeper_update(d);

        charging_timeout(&d->charging, &d->state);

        d->current_time = VESC_IF->system_time();
        
        imu_data_update(&d->imu, &d->balance_filter);
        motor_data_update(&d->motor);

        bool remote_connected = false;
        float servo_val = 0;

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

        if (!remote_connected) {
            servo_val = 0;
        } else {
            // Apply Deadband
            float deadband = d->float_conf.inputtilt_deadband;
            if (fabsf(servo_val) < deadband) {
                servo_val = 0.0;
            } else {
                servo_val = sign(servo_val) * (fabsf(servo_val) - deadband) / (1 - deadband);
            }

            // Invert Throttle
            servo_val *= (d->float_conf.inputtilt_invert_throttle ? -1.0 : 1.0);
        }

        d->throttle_val = servo_val;

        footpad_sensor_update(&d->footpad_sensor, &d->float_conf);

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
                float threshold = d->float_conf.tiltback_lv + 5;
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
                        d->float_conf.startup_pitch_tolerance + d->startup_pitch_trickmargin;
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
            apply_inputtilt(d);
            if (d->state.wheelslip) {
                atr_winddown(&d->atr);
                torque_tilt_winddown(&d->torque_tilt);
                turn_tilt_winddown(&d->turn_tilt);
                speed_tilt_winddown(&d->speed_tilt);
            } else {
                atr_update(&d->atr, &d->motor, &d->float_conf);
                torque_tilt_update(&d->torque_tilt, &d->motor, &d->float_conf);
                turn_tilt_update(&d->turn_tilt, &d->motor, &d->imu, &d->atr, &d->float_conf);
                speed_tilt_update(&d->speed_tilt, &d->motor, &d->float_conf);
            }

            aggregate_tiltbacks(d);

            pid_update(&d->pid, &d->imu, &d->motor, &d->float_conf, d->setpoint);
            set_current(d, d->pid.pid_value);
            break;

        case (STATE_READY):
            if (d->current_time - d->disengage_timer > 1800) {  // alert user after 30 minutes
                if (d->current_time - d->nag_timer > 60) {  // beep every 60 seconds
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
                d->startup_pitch_tolerance = d->float_conf.startup_pitch_tolerance;
            }

            check_odometer(d);

            // Check for valid startup position and switch state
            if (fabsf(d->imu.pitch_balance) < d->startup_pitch_tolerance &&
                fabsf(d->imu.roll) < d->float_conf.startup_roll_tolerance && is_engaged(d)) {
                reset_vars(d);
                break;
            }
            // Push-start aka dirty landing Part II
            if (d->float_conf.startup_pushstart_enabled && d->motor.erpm_abs > 1000 &&
                is_engaged(d)) {
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

        VESC_IF->sleep_us(d->loop_time_us);
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
    memcpy(buffer, &(d->float_conf), sizeof(RefloatConfig));
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

    read_cfg_from_eeprom(&d->float_conf);

    d->odometer = VESC_IF->mc_get_odometer();

    lcm_init(&d->lcm, &d->float_conf.hardware.leds);
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
        return d->pid.rate_p;
    case (5):
        return d->setpoint;
    case (6):
        return d->atr.interpolated;
    case (7):
        return d->motor.erpm;
    case (8):
        return d->motor.current;
    case (9):
        return d->motor.atr_filtered_current;
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
        d->float_conf.disabled = cfg[0] ? true : false;
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
        // temporarily reduce max currents to make hand test safer / gentler
        d->mc_current_max = d->mc_current_min = 7;
        // Disable I-term and all tune modifiers and tilts
        d->float_conf.ki = 0;
        d->float_conf.kp_brake = 1;
        d->float_conf.kp2_brake = 1;
        d->float_conf.torquetilt_strength = 0;
        d->float_conf.torquetilt_strength_regen = 0;
        d->float_conf.atr_strength_up = 0;
        d->float_conf.atr_strength_down = 0;
        d->float_conf.turntilt_strength = 0;
        // d->float_conf.speedtilt_constant = 0;
        d->float_conf.speedtilt_variable = 0;
        d->float_conf.fault_delay_pitch = 40;
        d->float_conf.fault_delay_roll = 40;
    } else {
        read_cfg_from_eeprom(&d->float_conf);
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
    buffer_append_float32_auto(buffer, d->throttle_val, &ind);

    if (d->state.state == STATE_RUNNING) {
        // Setpoints
        buffer_append_float32_auto(buffer, d->setpoint, &ind);

        buffer_append_float32_auto(buffer, d->atr.interpolated, &ind);
        buffer_append_float32_auto(buffer, d->atr.speed, &ind);
        buffer_append_float32_auto(buffer, d->atr.target, &ind);
        buffer_append_float32_auto(buffer, d->motor.erpm_abs_10k, &ind);
        buffer_append_float32_auto(buffer, d->torque_tilt.debug, &ind);
        // buffer_append_float32_auto(buffer, d->torque_tilt.interpolated, &ind);
        // buffer_append_float32_auto(buffer, d->turn_tilt.interpolated, &ind);
        // buffer_append_float32_auto(buffer, d->speed_tilt.interpolated, &ind);
        // buffer_append_float32_auto(buffer, d->inputtilt_interpolated, &ind);

        // DEBUG
        buffer_append_float32_auto(buffer, d->pid.pid_value, &ind);
        buffer_append_float32_auto(buffer, d->motor.atr_filtered_current, &ind);
        buffer_append_float32_auto(buffer, d->atr.accel_diff, &ind);
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
            send_buffer[ind++] = d->float_conf.hardware.leds.type;
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
            read_cfg_from_eeprom(&d->float_conf);
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
            lights_control_request(&d->float_conf.leds, &buffer[2], len - 2, &d->lcm);
            lights_control_response(&d->float_conf.leds);
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
        cfg = &d->float_conf;
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

    bool res = confparser_deserialize_refloatconfig(buffer, &d->float_conf);

    // Store to EEPROM
    if (res) {
        write_cfg_to_eeprom(d);
        configure(d);
        leds_configure(&d->leds, &d->float_conf.leds);
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

    if ((d->float_conf.is_beeper_enabled) ||
        (d->float_conf.inputtilt_remote_type != INPUTTILT_PPM)) {
        beeper_init();
    }

    balance_filter_init(&d->balance_filter);
    VESC_IF->imu_set_read_callback(imu_ref_callback);

    footpad_sensor_update(&d->footpad_sensor, &d->float_conf);

    d->main_thread = VESC_IF->spawn(refloat_thd, 1024, "Refloat Main", d);
    if (!d->main_thread) {
        log_error("Failed to spawn Refloat Main thread.");
        return false;
    }

    bool have_leds = leds_init(
        &d->leds, &d->float_conf.hardware.leds, &d->float_conf.leds, d->footpad_sensor.state
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
