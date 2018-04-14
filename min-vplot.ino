/* min-vplot: Minimal motion controller for v-plotter. Implements serial interface for NC. */

#include <EEPROM.h>
#include "TimerOne.h"

#include "geo.h"
#include "config.h"
#include "buffer.h"
#include "machine.h"
#include "gcode.h"
#include "parse.h"

machine_state current_state;


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
};


/* This function calculates destination data, stored in the current state, the speed for new moves and sets the motor speeds. */
void do_move(const cartesian_pt & next_pt)
{
  /* Disable the stepper ISR while we calculate. */
  noInterrupts();

  bool log_debug = false;

  if (log_debug)
  {
    Serial.print("New position: ");
    Serial.print(next_pt.x);
    Serial.print(" ");
    Serial.println(next_pt.y);
  }

  current_state.pt = next_pt;

  const plot_pos next_pos = pos_from_pt(next_pt);

  if (log_debug)
  {
    Serial.print("New pos: ");
    Serial.print(next_pos.a);
    Serial.print(" ");
    Serial.println(next_pos.b);
  }

  float da = next_pos.a - current_state.pos.a;
  float db = next_pos.b - current_state.pos.b;

  if (log_debug)
  {
    Serial.print("Change length: ");
    Serial.print(da);
    Serial.print(" ");
    Serial.println(db);
  }

  current_state.pos = next_pos;

  long new_a_steps = STEPS_PER_MM * next_pos.a;
  long new_b_steps = STEPS_PER_MM * next_pos.b;

  if (current_state.a_steps - new_a_steps == 0 && current_state.b_steps - new_b_steps == 0)
  {
    return;
  }

  current_state.a_dest = new_a_steps;
  current_state.b_dest = new_b_steps;

  if (log_debug)
  {
    Serial.print("Step dest: ");
    Serial.print(current_state.a_dest);
    Serial.print(" ");
    Serial.println(current_state.b_dest);
  }

  calculate_and_set_speed_ratio(da, db);

  if (log_debug)
  {
    Serial.print("A speed: ");
    Serial.print(current_state.motor_a.getSpeed());
    Serial.print(" B speed: " );
    Serial.println(current_state.motor_b.getSpeed());
  }

 interrupts();
}

void calculate_and_set_speed_ratio(float da, float db)
{
  /* Calculate feeds such that we arrive at the end of the segment on both axes simultaneously.
   * This is now called from prepare_motion to correct the difference between cartesian and kinematic lines.
   * TODO: Improve to use Bresenham. */

  float a_speed = current_state.feed;
  float b_speed = current_state.feed;

  long a_current_steps = current_state.motor_a.getPositionSteps();
  long b_current_steps = current_state.motor_b.getPositionSteps();

  long da_steps = abs(current_state.a_dest - a_current_steps);
  long db_steps = abs(current_state.b_dest - b_current_steps);

  if (abs(da) > abs(db))
  {
    b_speed = a_speed * abs(db / da);

    if (abs(db_steps) > 0)
    {
      b_speed = max(b_speed, 1.0);
    }
  }
  else
  {
    a_speed = b_speed * abs(da / db);

    if (abs(da_steps) > 0)
    {
      a_speed = max(a_speed, 1.0);
    }
  }

  a_speed = da > 0 ? a_speed : -a_speed;
  b_speed = db > 0 ? b_speed : -b_speed;

  current_state.motor_a.setSpeed(a_speed);
  current_state.motor_b.setSpeed(b_speed);
}

void do_lift(bool lift)
{
  bool log_debug = false;

  if (log_debug)
  {
    Serial.print("Servo: ");
    Serial.println(lift ? SERVO_LIFT_POSITION : 0);
  }

  current_state.lift = lift;
  current_state.servo.moveToDegrees(lift ? SERVO_LIFT_POSITION : 0);
}

/* Prepare motion from the buffer; if we have arrived at our end point, start the next move. */

/* This is called from the main loop, so there is a certain amount of delay from the end of
 * one movement to the start of another (for instance, while reading serial inputs or calculating
 * speeds below). It would be better if the stepper interrupt was allowed to advance to the next move. */

void prepare_motion()
{
  // why is ustepper disabling?
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  long a_current_steps = current_state.motor_a.getPositionSteps();
  long b_current_steps = current_state.motor_b.getPositionSteps();

  if (a_current_steps == current_state.a_dest && b_current_steps == current_state.b_dest)
  {
    if (!get_buffer_empty())
    {
      bool was_full = get_buffer_full();
      gc_block block = buffer_advance();

      if (block.pt.x != current_state.pt.x || block.pt.y != current_state.pt.y)
      {
         do_move(block.pt);
      }
      else if (block.lift != current_state.lift)
      {
        do_lift(block.lift);
        delay(1000); /* Wait for motion to complete (interrupt still fires) */

        /* TODO: Avoid updating servo outside of delay to eliminate jitter. */
      }
      else
      {
        current_state.feed = block.feed;
      }

      if (was_full)
        Serial.println("ok"); // let connection know we are ready for more input
    }
  }
  else /* moving to destination */
  {
    /* Moving the steppers at constant speeds creates an arc in cartesian space, so
     * we must constantly re-calculate speeds to keep us on a straight cartesian line.
     *
     * This method creates a vector from the current plotter cartesian position to the
     * commanded cartesian target point, creates a target point for setting speeds a short
     * distance away to minimize error (here 1mm), then sets speeds to arrive at that point.
     *
     * If the length to destination is less than 1mm, our error will be minimal so skip.
     *
     * float divide/sqrt is ~500 avr clock cycles. (0.03125ms @ 16Mhz?)
     * I count 13 divides/mults and 4 sqrts in these calculations for perhaps .5 ms total.
     */

    const plot_pos pos = current_state.get_current_plot_pos();
    const cartesian_pt pt = current_state.get_current_cartesian_location();

    const cartesian_vec vec(current_state.pt.x /* g-code target */ - pt.x, current_state.pt.y - pt.y);
    const float vec_length = sqrt(vec.x * vec.x + vec.y * vec.y);

    if (vec_length > 1.0)
    {
      const cartesian_vec norm_vec(vec.x / vec_length, vec.y / vec_length);

      cartesian_pt speed_target_pt(norm_vec.x + pt.x, norm_vec.y + pt.y);
      plot_pos speed_target_pos = pos_from_pt(speed_target_pt);

      calculate_and_set_speed_ratio(speed_target_pos.a - pos.a, speed_target_pos.b - pos.b);

      bool log_debug = false;
      if (log_debug)
      {
        Serial.print(pos.a);
        Serial.print(" ");
        Serial.print(pos.b);
        Serial.print(" ");
        Serial.print(pt.x);
        Serial.print(" ");
        Serial.print(pt.y);
        Serial.print(" ");
        Serial.print(norm_vec.x);
        Serial.print(" ");
        Serial.print(norm_vec.y);
        Serial.print(" ");
        Serial.print("A speed: ");
        Serial.print(current_state.motor_a.getSpeed());
        Serial.print(" B speed: " );
        Serial.println(current_state.motor_b.getSpeed());
      }
    }
  }
}

static char line[128];

/* Main loop: parse serial inputs and prepare motion from the buffer. */
void loop()
{
  char c;
  uint8_t char_counter = 0;

  while (true)
  {
    if ((c = Serial.read()) != -1)
    {
      if ((c == '\n') || (c == '\r')) // End of line reached
      {
        line[char_counter] = 0; // Set string termination character.

        if (char_counter > 0)
        {
          //Serial.println(line);
          parse_line(line, current_state);

          char_counter = 0;
        }

        memset(line, 0, 128);
      }
      else
      {
        line[char_counter++] = c;
      }
    }

    prepare_motion();

    Serial.flush();
  }
}

/* Main interrupt routine, drives steppers and servo. Called every INTERRUPT_PERIOD_US. */
void stepper_isr()
{
  if (current_state.motor_a.getPositionSteps() != current_state.a_dest)
    current_state.motor_a.step();

  if (current_state.motor_b.getPositionSteps() != current_state.b_dest)
    current_state.motor_b.step();

   current_state.servo.update();
}

void setup()
{
  Timer1.initialize(INTERRUPT_PERIOD_US);
  Timer1.attachInterrupt(stepper_isr);

  /* Reset the servo position; upon startup, the horn should be pushing the pen tip off the surface. */
  current_state.servo.moveToDegrees(0);
  delay(1000);
  current_state.servo.moveToDegrees(SERVO_LIFT_POSITION);

  Serial.begin(115200);
  Serial.setTimeout(0); /* This allows us to prepare the next move without blocking on inputs. */

  while (!Serial)
    delay(10); // for arduino micro

  while (true)
  {
    char init = Serial.read();

    if (init == '>')
    {
      Serial.println("Ready");
      break;
    }

    delay(100);
  }

  current_state.motor_a.enable();
  current_state.motor_b.enable();

  current_state.motor_a.setSpeed(0);
  current_state.motor_b.setSpeed(0);

  current_state.motor_a.setPosition(current_state.a_steps);
  current_state.motor_b.setPosition(current_state.b_steps);
}
