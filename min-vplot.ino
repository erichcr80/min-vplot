#include "uStepper.h"
#include "grbl_read_float.h"

uStepper motor( 80, 0, 80000, 11, 12, 2);
uStepper motor2( 80, 1, 80000, 10, 13, 2);

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

float max_speed = 3200 / 4;

float a_speed = 0;
float b_speed = 0;

long a_dest = 0; // 200L * 16L * 2L;
long b_dest = 0; // 200L * 16L;

void do_move(float new_x, float new_y)
{
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
}

static char line[255];

void setup()
{
  Serial.begin(9600);
  // put your setup code here, to run once:

  motor.enable();
  motor.setSpeed(0);

  motor2.enable();
  motor2.setSpeed(0);

  Serial.print("Ready: ");

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

        byte loc = 0;
        float val = 0.0;
        
        if (read_float(line, &loc, &val))
        {
          Serial.print(val);
        }
        else
        {
          Serial.println("Not a float");
        }       
      }
      else 
      {
        line[char_counter++] = c;   
        //Serial.print(c); 
      }
    }
  }
}

bool has_direction = false;
bool has_amount = false;

int incoming_direction = 0;
int incoming_amount = 0;

class box_draw
{
    int move_count = 0;

    float x = 0.0;
    float y = 0.0;

  public:
    int segments;
    float side_length; // mm

    box_draw(int _segments, float _side_length)
    {
      segments = _segments;
      side_length = _side_length;
    }

    bool done()
    {
      return move_count > segments * 4 + 1;
    }

    void do_next()
    {
      if (move_count == 0)
      {
        x = side_length / 2.0;
        y = side_length / 2.0;

        do_move(x, y);
        move_count++;
        return;
      }

      if (move_count == segments * 4)
      {
        do_move(side_length / 2.0, side_length / 2.0);
        move_count++;
        return;
      }
      else if (move_count > segments * 4)
      {
        do_move(0, 0);
        move_count++;
        return;
      }

      if (move_count <= segments)
      {
        y -= side_length / (float)segments;
      }
      else if (move_count <= segments * 2)
      {
        x -= side_length / (float)segments;
      }
      else if (move_count <= segments * 3)
      {
        y += side_length / (float)segments;
      }
      else if (move_count <= segments * 4)
      {
        x += side_length / (float)segments;
      }

      do_move(x, y);
      move_count++;
    }
};

box_draw bd(2, 20.0);

bool drawing_box = false;

void loop()
{
  bool motor_1_running = motor.getPositionSteps() != a_dest;
  bool motor_2_running = motor2.getPositionSteps() != b_dest;

  if (motor_1_running)
    motor.step();

  if (motor_2_running)
    motor2.step();

  // Serial.println(motor_1_running);
  // Serial.println(motor_2_running);


  if (!motor_1_running && !motor_2_running && drawing_box)
  {
    if (!bd.done())
    {
      bd.do_next();
    }
    else
    {

      bd = box_draw(bd.segments * 1.5, bd.side_length + 10);
      //bd = box_draw(64, 100.0);


      /*
        Serial.print("New box: " );
        Serial.print(bd.segments);
        Serial.print(" " );
        Serial.println(bd.side_length);
      */

      bd.do_next();
    }
  }

  delayMicroseconds(12);

  /*
    Serial.print("Step1:");
    Serial.print(motor.getPositionSteps());
    Serial.print("\tStep2:");
    Serial.println(motor2.getPositionSteps());
  */

  // send data only when you receive data:
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
}
