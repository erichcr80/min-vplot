#include "uStepper.h"
#include "TimerOne.h"
#include "grbl_read_float.h"

uStepper motor( 80, 0, 10000, 11, 12, 2);
uStepper motor2( 80, 1, 10000, 10, 13, 2);

float stepper_distance = 500; // mm

float origin_x = stepper_distance / 2.0;
float origin_y = stepper_distance / 2.0;

float a_length_from_xy(float x, float y)
{
  float dx = x - origin_x;
  float dy = y - origin_y;

  return sqrt(dx * dx + dy * dy);
}

float b_length_from_xy(float x, float y)
{
  float dx = x + origin_x;
  float dy = y - origin_y;

  return sqrt(dx * dx + dy * dy);
}

float dlength_to_steps(float dlength)
{
  return dlength / .188496 * 16; // mm / step
}

float a = stepper_distance * sqrt(2) / 2;
float b = stepper_distance * sqrt(2) / 2;

float x = 0.0;
float y = 0.0;

float da = 0.0;
float db = 0.0;

float max_speed = 200;

float a_speed = 0;
float b_speed = 0;

volatile long a_dest = 0; // 200L * 16L * 2L;
volatile long b_dest = 0; // 200L * 16L;

void do_move(float new_x, float new_y)
{
  noInterrupts();
  Timer1.stop();
  
  bool log_debug = false;

  if (log_debug)
  {
    Serial.print("New position: ");
    Serial.print(new_x);
    Serial.print(" ");
    Serial.println(new_y);
  }

  float new_a = a_length_from_xy(new_x, new_y);
  float new_b = b_length_from_xy(new_x, new_y);

  if (log_debug)
  {
    Serial.print("New length: ");
    Serial.print(new_a);
    Serial.print(" ");
    Serial.println(new_b);
  }

  da = new_a - a;
  db = new_b - b;

  if (log_debug)
  {
    Serial.print("Change length: ");
    Serial.print(da);
    Serial.print(" ");
    Serial.println(db);
  }

  a = new_a;
  b = new_b;

  x = new_x;
  y = new_y;

  float steps_a = dlength_to_steps(da);
  float steps_b = dlength_to_steps(db);

  if (log_debug)
  {
    Serial.print("Change length (steps): ");
    Serial.print(steps_a);
    Serial.print(" ");
    Serial.println(steps_b);
  }

  a_dest = steps_a + motor.getPositionSteps();
  b_dest = steps_b + motor2.getPositionSteps();
  
  a_speed = max_speed;
  b_speed = max_speed;

  if (abs(da) > abs(db))
  {
    b_speed = max_speed * abs(db / da);
  }
  else
  {
    a_speed = max_speed * abs(da / db);
  }

  a_speed = da > 0 ? a_speed : -a_speed;
  b_speed = db > 0 ? b_speed : -b_speed;

  motor.setSpeed(a_speed);
  motor2.setSpeed(b_speed);
 
  if (log_debug)
  {
    Serial.print("A speed: ");
    Serial.print(a_speed);
    Serial.print(" B speed: " );
    Serial.println(b_speed);
  }

 Timer1.resume();
 interrupts();
}

static char line[64];

void parse_line()
{
  Serial.print("Parsing line:");
  Serial.println(line);
  
  uint8_t char_counter = 0;

  while (line[char_counter] != 0)
  {
    if (line[char_counter] == ' ')
    {
      char_counter++;
      continue;
    }
    
    char letter = line[char_counter];
    char_counter++;
    
    float value = 0.0;
    if (read_float(line, &char_counter, &value))
    {
      Serial.print("Letter: ");
      Serial.print(letter);
      Serial.print(", Value: " );
      Serial.println(value);
      continue;
    }
    else
    {
      Serial.println("Not a float");
    }   

    char_counter++;
  }
}

void receive_loop()
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
        char_counter = 0;

        parse_line();
      }
      else 
      {
        line[char_counter++] = c;   
      }
    }

    /*
    long m1steps = motor.getPositionSteps();
    long m2steps = motor2.getPositionSteps();
    
    bool motor_1_running = m1steps != a_dest;
    bool motor_2_running = m2steps != b_dest;
    */

    
  }
}

void setup()
{
  Serial.begin(9600);
  // put your setup code here, to run once:

  motor.enable();
  motor.setSpeed(0);

  motor2.enable();
  motor2.setSpeed(0);

  Serial.print("Ready: ");

  Timer1.initialize(100); // set a timer of length 100 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( my_loop ); // attach the service routine here

  receive_loop();
}

bool has_direction = false;
bool has_amount = false;

int incoming_direction = 0;
int incoming_amount = 0;

void my_loop()
{
  if (motor.getPositionSteps() != a_dest)
    motor.step();

  if (motor2.getPositionSteps() != b_dest)
    motor2.step();

  // Serial.println(motor_1_running);
  // Serial.println(motor_2_running);

  /*
    Serial.print("Step1:");
    Serial.print(motor.getPositionSteps());
    Serial.print("\tStep2:");
    Serial.println(motor2.getPositionSteps());
  */

  // send data only when you receive data:

  /*
  if (Serial.available() > 0)
  {
    byte val = Serial.read();

    if (!has_direction)
    {
      incoming_direction = val - 48;
      has_direction = true;
    }
    else
    {
      incoming_amount = val - 48;

      Serial.print("I received: ");
      Serial.print(incoming_direction, DEC);
      Serial.print(" ");
      Serial.println(incoming_amount, DEC);

      has_amount = false;
      has_direction = false;

      if (incoming_direction == 1)
      {
        motor.setSpeed(max_speed);
        a_dest += incoming_amount * 1000;
      }
      else if (incoming_direction == 2)
      {
        motor.setSpeed(-max_speed);
        a_dest -= incoming_amount * 1000;
      }
      else if (incoming_direction == 3)
      {
        motor2.setSpeed(max_speed);
        b_dest += incoming_amount * 1000;
      }
      else if (incoming_direction == 4)
      {
        motor2.setSpeed(-max_speed);
        b_dest -= incoming_amount * 1000;
      }
      else if (incoming_direction == 5)
      {
        drawing_box = true;

        motor.setPosition(0);
        motor2.setPosition(0);
      }
    }
  }
  */
}
