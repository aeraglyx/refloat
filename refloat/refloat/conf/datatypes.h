// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    INPUTTILT_NONE = 0,
    INPUTTILT_UART,
    INPUTTILT_PPM
} FLOAT_INPUTTILT_REMOTE_TYPE;

typedef enum {
    LED_TYPE_NONE = 0,
    LED_TYPE_RGB,
    LED_TYPE_RGBW,
    LED_TYPE_EXTERNAL,
} LedType;

typedef enum : uint8_t {
    LED_PIN_B6 = 0,
    LED_PIN_B7
} LedPin;

typedef enum : uint8_t {
    COLOR_BLACK = 0,
    COLOR_WHITE_FULL,
    COLOR_WHITE_RGB,
    COLOR_WHITE_SINGLE,
    COLOR_RED,
    COLOR_FERRARI,
    COLOR_FLAME,
    COLOR_CORAL,
    COLOR_SUNSET,
    COLOR_SUNRISE,
    COLOR_GOLD,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_BANANA,
    COLOR_LIME,
    COLOR_ACID,
    COLOR_SAGE,
    COLOR_GREEN,
    COLOR_MINT,
    COLOR_TIFFANY,
    COLOR_CYAN,
    COLOR_STEEL,
    COLOR_SKY,
    COLOR_AZURE,
    COLOR_SAPPHIRE,
    COLOR_BLUE,
    COLOR_VIOLET,
    COLOR_AMETHYST,
    COLOR_MAGENTA,
    COLOR_PINK,
    COLOR_FUCHSIA,
    COLOR_LAVENDER,
} LedColor;

typedef enum : uint8_t {
    LED_MODE_SOLID = 0,
    LED_MODE_FADE,
    LED_MODE_PULSE,
    LED_MODE_STROBE,
    LED_MODE_KNIGHT_RIDER
} LedMode;

typedef enum : uint8_t {
    LED_TRANS_FADE = 0,
    LED_TRANS_FADE_OUT_IN,
    LED_TRANS_CIPHER,
    LED_TRANS_MONO_CIPHER,
} LedTransition;

typedef struct {
    float brightness;
    LedColor color1;
    LedColor color2;
    LedMode mode;
    float speed;
} LedBar;

typedef struct {
    uint16_t idle_timeout;
    float duty_threshold;
    float red_bar_percentage;
    bool show_sensors_while_running;
    float brightness_headlights_on;
    float brightness_headlights_off;
} StatusBar;

typedef struct {
    bool on;
    bool headlights_on;

    LedTransition headlights_transition;
    LedTransition direction_transition;

    bool lights_off_when_lifted;
    bool status_on_front_when_lifted;

    LedBar headlights;
    LedBar taillights;
    LedBar front;
    LedBar rear;
    StatusBar status;
    LedBar status_idle;
} CfgLeds;

typedef struct {
    uint8_t count;
    bool reverse;
} CfgLedStrip;

typedef struct {
    LedType type;
    LedPin pin;
    CfgLedStrip status;
    CfgLedStrip front;
    CfgLedStrip rear;
} CfgHwLeds;

typedef struct {
    FLOAT_INPUTTILT_REMOTE_TYPE remote_type;
    bool invert_throttle;
    float deadband;
} CfgHwRemote;

typedef struct {
    uint16_t hertz;
    bool is_beeper_enabled;
} CfgHwEsc;

typedef struct {
    CfgHwEsc esc;
    CfgHwRemote remote;
    CfgHwLeds leds;
} CfgHardware;

// typedef struct {
//     float angle;
//     float speed;
//     float start;
// } Tiltback;

typedef struct {
    float threshold;
    float tiltback_angle;
    float tiltback_speed;
    // TODO haptic buzz
} Warning;

typedef struct {
    Warning duty;
    Warning lv;
    Warning hv;
    float tiltback_return_speed;

    bool is_dutybeep_enabled;
    bool is_footbeep_enabled;
    bool is_surgebeep_enabled;
} CfgWarnings;

// typedef struct {
//     float tiltback_duty_angle;
//     float tiltback_duty_speed;
//     float tiltback_duty;
//     float tiltback_hv_angle;
//     float tiltback_hv_speed;
//     float tiltback_hv;
//     float tiltback_lv_angle;
//     float tiltback_lv_speed;
//     float tiltback_lv;
//     float tiltback_return_speed;
// } CfgWarnings;

typedef struct {
    float pitch;
    float roll_threshold;
    float adc1_threshold;
    float adc2_threshold;
    uint16_t pitch_delay;
    uint16_t roll_delay;
    uint16_t switch_half_delay;
    uint16_t switch_full_delay;
    uint16_t switch_half_erpm;
    bool is_posi_enabled;
    bool fault_moving_fault_disabled;
    bool fault_reversestop_enabled;
} CfgFaults;

typedef struct {
    float mahony_kp;
    float mahony_kp_roll;
    float mahony_kp_yaw;
    float bf_accel_confidence_decay;
} CfgBalanceFilter;

typedef struct {
    float kp;
    float kd;
    float kd_filter;
    float ki;
    float ki_limit;
    float kp_brake;
    float kd_brake;
} CfgPid;

typedef struct {
    float strength_up;
    float strength_down;
    float strength_boost;
    float threshold;
    float angle_limit;
    float speed;
    float speed_max;
    float speed_boost;
    float transition_boost;
    float amps_accel_ratio;
    float amp_offset_constant;
    float amp_offset_variable;
    float filter;
} CfgAtr;

typedef struct {
    float strength;
    float strength_regen;
    float strength_boost;
    float start_current;
    float angle_limit;
    float speed;
    float speed_max;
    float turn_boost;
    float method;
    float filter;
} CfgTorqueTilt;

typedef struct {
    float strength;
    float strength_boost;
    float angle_limit;
    float speed;
    float speed_max;
    float start_angle;
    uint16_t start_erpm;
    float filter;
} CfgTurnTilt;

typedef struct {
    float constant;
    float variable;
    float variable_max;
    float speed;
    float speed_max;
} CfgSpeedTilt;

typedef struct {
    float speed;
    float speed_max;
    float angle_limit;
    float filter;
    float remote_throttle_current_max;
    float remote_throttle_grace_period;
} CfgInputTilt;

typedef struct {
    CfgBalanceFilter balance_filter;
    CfgPid pid;
    CfgAtr atr;
    CfgTorqueTilt torque_tilt;
    CfgTurnTilt turn_tilt;
    CfgSpeedTilt speed_tilt;
    CfgInputTilt input_tilt;
} CfgTune;

typedef struct {
    float version;
    bool disabled;

    // uint16_t hertz;

    CfgHardware hardware;

    CfgTune tune;
    CfgFaults faults;
    CfgWarnings warnings;

    CfgLeds leds;

    float startup_pitch_tolerance;
    float startup_roll_tolerance;
    float startup_speed;
    bool startup_simplestart_enabled;
    bool startup_pushstart_enabled;
    bool startup_dirtylandings_enabled;

    float brake_current;

    float surge_duty_start;
    float surge_angle;
} RefloatConfig;

// DATATYPES_H_
#endif
