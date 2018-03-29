#pragma once

#include <RBD_Servo.h>
#include "uStepper.h"

#include "config.h"
#include "geo.h"

/* gcode & machine state object; contains current movement parameters and state for the plotter. */
class machine_state
{
public:
  int feed = MAX_FEED;

  bool rapid = false;

  cartesian_pt pt;
  plot_pos pos;

  long a_steps = 0L;
  long b_steps = 0L;

  long a_dest = 0L;
  long b_dest = 0L;

  bool lift = true;

  machine_state() :
     pos(STEPPER_DISTANCE_MM * sqrt(2.0) / 2.0,
         STEPPER_DISTANCE_MM * sqrt(2.0) / 2.0)
  {
    a_steps = STEPS_PER_MM * pos.a;
    b_steps = STEPS_PER_MM * pos.b;

    a_dest = a_steps;
    b_dest = b_steps;
  }
} current_state;

uStepper motor_a(STEPS_PER_MM, 0, 1000000 /* us / s */ / INTERRUPT_PERIOD_US, 4, 3, ENABLE_PIN);
uStepper motor_b(STEPS_PER_MM, 1, 1000000 /* us / s */ / INTERRUPT_PERIOD_US, 8, 7, ENABLE_PIN);

/* Servo object */
RBD::Servo servo(2 /* pin */, 1000 /* pulse_min */, 2750 /* pulse_max */);

/* Inverse kinematics: Calculate plotter position from cartesian coordinates. */
plot_pos pos_from_pt(const cartesian_pt & pt)
{
  float dxa = pt.x - ORIGIN_X;
  float dxb = pt.x + ORIGIN_X;

  float dy = pt.y - ORIGIN_Y;

  return plot_pos(
    sqrt(dxa * dxa + dy * dy),
    sqrt(dxb * dxb + dy * dy)
  );
}

plot_pos get_current_plot_pos()
{
  return plot_pos(
    motor_a.getPositionSteps() / (STEPS_PER_MM),
    motor_b.getPositionSteps() / (STEPS_PER_MM));
}

/* Forward kinematics: Calculate cartesian coordinates from current plotter position.
   Equation from: http://www.diale.org/vbot.html */
cartesian_pt get_current_cartesian_location()
{
  const plot_pos current_pos = get_current_plot_pos();

  const float b_p_a_sq = current_pos.b * current_pos.b + current_pos.a * current_pos.a;
  const float b_m_a_sq = current_pos.b * current_pos.b - current_pos.a * current_pos.a;

  const float x = b_m_a_sq / (2.0 * STEPPER_DISTANCE_MM);
  const float y = ORIGIN_Y - (1.0 / SQRT_2) *
    sqrt(b_p_a_sq -
      (STEPPER_DISTANCE_MM * STEPPER_DISTANCE_MM / 2.0) -
      ((b_m_a_sq * b_m_a_sq) / (2.0 * STEPPER_DISTANCE_MM * STEPPER_DISTANCE_MM)));

  return cartesian_pt(x, y);
}
