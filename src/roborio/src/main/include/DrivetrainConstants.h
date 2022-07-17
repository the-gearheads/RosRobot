#pragma once

#include <units/constants.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/velocity.h>

#define RF_MOTOR_ID 4
#define RB_MOTOR_ID 5
#define LF_MOTOR_ID 6
#define LB_MOTOR_ID 7

#define TRACK_WIDTH 0.622_m

#define TALON_UNITS_PER_ROTATION 2048
#define SHAFT_TO_WHEEL_GEAR_RATIO 12.75
#define WHEEL_RADIUS 4_in
#define WHEEL_CIRCUMFERENCE (2 * WHEEL_RADIUS * units::constants::pi)

#define L_FF_KS 0.5_V
#define L_FF_KV (2.973787777757068_V / 1_mps)

#define R_FF_KS 0.5_V
#define R_FF_KV (2.973787777757068_V / 1_mps)

#define SIM_ROBOT_MASS 56_kg

#define SIM_LINEAR_KV (1.9817_V / 1_mps)
#define SIM_LINEAR_KA (0.3182_V / 1_mps_sq)

#define SIM_ANGULAR_KV (1.79_V / 1_mps)
#define SIM_ANGULAR_KA (0.69482_V / 1_mps_sq)
