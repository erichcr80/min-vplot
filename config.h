/* min-vplot: Minimal motion controller for v-plotter. */

#pragma once

#define STEPPER_DISTANCE_MM 1800.0
#define SQRT_2 1.41421356237

#define ORIGIN_X STEPPER_DISTANCE_MM / 2.0 /* Assumes cold start always at X= middle point of motors and Y= 1/2 motor distance down. */
#define ORIGIN_Y STEPPER_DISTANCE_MM / 2.0

#define PULLEY_RADIUS_MM 6.3662
#define PULLEY_CIRCUMFERENCE TWO_PI * PULLEY_RADIUS_MM
#define STEPS_PER_REVOLUTION 200.0
#define MICROSTEP_RESOLUTION 16.0

#define STEPS_PER_MM 1.0 / (PULLEY_CIRCUMFERENCE / STEPS_PER_REVOLUTION / MICROSTEP_RESOLUTION)

#define MAX_FEED 800.0
#define INTERRUPT_PERIOD_US 50

#define SERVO_LIFT_POSITION 90

/* TODO: Interrupt period could be calculated from the maximum feed vs steps per MM. */

/* Stepper objects */
#define ENABLE_PIN 6
