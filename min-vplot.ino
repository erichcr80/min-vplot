/* min-vplot: Minimal motion controller for v-plotter. Implements serial interface for NC. */

#include <EEPROM.h>
#include <RBD_Servo.h>
#include "uStepper.h"
#include "TimerOne.h"

#include "grbl_read_float.h" // from grbl

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

uStepper motor_a(STEPS_PER_MM, 0, 1000000 /* us / s */ / INTERRUPT_PERIOD_US, 4, 3, ENABLE_PIN);
uStepper motor_b(STEPS_PER_MM, 1, 1000000 /* us / s */ / INTERRUPT_PERIOD_US, 8, 7, ENABLE_PIN);

/* Servo object */
RBD::Servo servo(2, 1000, 2750);

struct cartesian_pt
{
  float x = 0.0;
  float y = 0.0;

  cartesian_pt() : x(0.0), y(0.0) {}
  cartesian_pt(const float x, const float y) : x(x), y(y) {}
};

typedef cartesian_pt cartesian_vec;

struct plot_pos
{
  float a = 0.0;
  float b = 0.0;

  plot_pos() : a(0.0), b(0.0) {}
  plot_pos(const float a, const float b) : a(a), b(b) {}
};

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

/* gcode block object; contains movement and state change data. Parsed from serial and stored in buffer. */
class gc_block
{
public:
  cartesian_pt pt;
  
  int feed = MAX_FEED;
  bool lift = true;

  gc_block() {}
  gc_block(float x, float y, bool lift) : pt(x, y), lift(lift) {}
};

/* gcode & machine state object; contains current movement parameters and state for the plotter. */
class gc_state
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

  gc_state() : 
     pos(STEPPER_DISTANCE_MM * sqrt(2.0) / 2.0, 
         STEPPER_DISTANCE_MM * sqrt(2.0) / 2.0)
  {
    a_steps = STEPS_PER_MM * pos.a;
    b_steps = STEPS_PER_MM * pos.b;

    a_dest = a_steps;
    b_dest = b_steps;
  }
};

#define BUFFER_SIZE 5
gc_block gc_buffer[BUFFER_SIZE];

int buffer_front = 0;
int buffer_back = 0;

int get_buffer_front()
{
  return buffer_front;
}

bool get_buffer_empty()
{
  return buffer_front == buffer_back;
}

bool get_buffer_full()
{
  return buffer_front == get_next();
}

int get_next()
{
  if (buffer_back >= BUFFER_SIZE - 1)
    return 0;
  else
    return buffer_back + 1;
}

gc_block buffer_advance();
gc_block buffer_advance()
{
  int current = buffer_front;
  buffer_front++;
  if (buffer_front >= BUFFER_SIZE)
    buffer_front = 0;
  return gc_buffer[current];
}

gc_block & buffer_current();
gc_block & buffer_current()
{
  return gc_buffer[buffer_front];
}

gc_block & buffer_last();
gc_block & buffer_last()
{
  return gc_buffer[buffer_back - 1 < 0 ? BUFFER_SIZE - 1 : buffer_back - 1];
}

bool buffer_add(gc_block gc_add);
bool buffer_add(gc_block gc_add)
{
  if (get_buffer_full())
    return false;

  int next = buffer_back;
  gc_buffer[next] = gc_add;
  
  buffer_back = get_next();
  
  return true;
}

gc_state current_state;

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
    Serial.print(motor_a.getSpeed());
    Serial.print(" B speed: " );
    Serial.println(motor_b.getSpeed());
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

  long a_current_steps = motor_a.getPositionSteps();
  long b_current_steps = motor_b.getPositionSteps();

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

  motor_a.setSpeed(a_speed);
  motor_b.setSpeed(b_speed);
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
  servo.moveToDegrees(lift ? SERVO_LIFT_POSITION : 0);
}

static char line[128];

void parse_line()
{ 
  uint8_t char_counter = 0;
  bool movement = false;
  bool comment = false;

  gc_block current_block = buffer_last();
  
  while (line[char_counter] != 0)
  {
    if (line[char_counter] == ' ' || line[char_counter] == '\r' || line[char_counter] == '\n')
    {
      char_counter++;
      continue;
    }

    if (line[char_counter] == '(' || comment)
    {
      comment = true;
      char_counter++;
      continue;
    }

    if (line[char_counter] == ')')
    {
      char_counter++;
      comment = false;
      continue;
    }
    
    char letter = line[char_counter];
    char_counter++;
    
    float value = 0.0;
    if (read_float(line, &char_counter, &value))
    {
      if (value > 1e9 || value < -1e9 || value == NAN)
      {
        Serial.print("Parsing error: ");
        Serial.println(value);
      }
      
      if (letter == 'G')
      {
        if (value == 1)
        {
          current_state.rapid = true;
        }
        else if (value == 0)
        {
          current_state.rapid = false;
        }
      }
      else if (letter == 'M')
      {
        if (value == 0.0) /* diagnostic printout */
        {
          Serial.print(current_state.a_dest);
          Serial.print(" ");    
          Serial.print(motor_a.getSpeed());
          Serial.print(" ");
          Serial.print(current_state.b_dest);
          Serial.print(" "); 
          Serial.print(motor_b.getSpeed());
          Serial.print(" ");  
          Serial.print(motor_a.getPositionSteps());
          Serial.print(" ");          
          Serial.println(motor_b.getPositionSteps());
        }
        else
        {         
          current_block.lift = value == 3;
        }
      }
      else if (letter == 'F')
      {
        current_block.feed = min((int)value, MAX_FEED);
      }
      else if (letter == 'X')
      {
        current_block.pt.x = value;
      }
      else if (letter == 'Y')
      {
        current_block.pt.y = value;
      }
          
      continue;
    }
    else
    {
      Serial.print("Invalid number: ");
      Serial.println(line);
      return;
    }   

    char_counter++;
  }

  if (!get_buffer_full())
  {
    buffer_add(current_block);

    if (!get_buffer_full())
      Serial.println("ok");
  }
  else
  {
    Serial.println("dropped");
  }
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

  long a_current_steps = motor_a.getPositionSteps();
  long b_current_steps = motor_b.getPositionSteps();
  
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

    const plot_pos pos = get_current_plot_pos();
    const cartesian_pt pt = get_current_cartesian_location();

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
        Serial.print(motor_a.getSpeed());
        Serial.print(" B speed: " );
        Serial.println(motor_b.getSpeed()); 
      }        
    }
  }
}

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
          parse_line();
          
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
  if (motor_a.getPositionSteps() != current_state.a_dest)
    motor_a.step();

  if (motor_b.getPositionSteps() != current_state.b_dest)
    motor_b.step();

   servo.update();
}

void setup()
{
  Timer1.initialize(INTERRUPT_PERIOD_US);
  Timer1.attachInterrupt(stepper_isr);

  /* Reset the servo position; upon startup, the horn should be pushing the pen tip off the surface. */
  servo.moveToDegrees(0);
  delay(1000);
  servo.moveToDegrees(SERVO_LIFT_POSITION);

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

  motor_a.enable();
  motor_b.enable();

  motor_a.setSpeed(0);
  motor_b.setSpeed(0);  

  motor_a.setPosition(current_state.a_steps);
  motor_b.setPosition(current_state.b_steps);
}

