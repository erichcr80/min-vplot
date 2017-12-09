#include <RBD_Servo.h>

#include "uStepper.h"
#include "TimerOne.h"
#include "grbl_read_float.h"

uStepper motor( 80, 0, 20000, 4, 3, 6);
uStepper motor2( 80, 1, 20000, 8, 7, 6);

RBD::Servo servo(2, 1000, 2750);

float stepper_distance = 1000; // mm

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

long length_to_steps(float length)
{ 
  return length / .188496 * 16; // mm / step
}

float da = 0.0;
float db = 0.0;

float max_speed = 800;

float a_speed = 0; 
float b_speed = 0;

volatile long a_dest = 0;
volatile long b_dest = 0;

class gc_block
{
public:
  float x = 0.0;
  float y = 0.0;

  bool lift = true;
  bool ack = false;

  gc_block() {}
  gc_block(float _x, float _y, bool _lift) : x(_x), y(_y), lift(_lift) {}
};

class gc_state
{
public:
  int feed = 0;
    
  bool rapid = false;

  float x = 0.0;
  float y = 0.0;

  float a_length = 0.0;
  float b_length = 0.0;
  
  long a_steps = 0L;
  long b_steps = 0L;
    
  bool lift = true;

  gc_state()
  {
    a_length = stepper_distance * sqrt(2) / 2;
    b_length = stepper_distance * sqrt(2) / 2;   

    a_steps = length_to_steps(a_length);
    b_steps = length_to_steps(b_length);

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

void do_move(float new_x, float new_y)
{
  noInterrupts();

  bool log_debug = false;

  if (log_debug && false)
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

  da = new_a - current_state.a_length;
  db = new_b - current_state.b_length;

  if (log_debug)
  {
    Serial.print("Change length: ");
    Serial.print(da);
    Serial.print(" ");
    Serial.println(db);
  }

  current_state.a_length = new_a;
  current_state.b_length = new_b;

  current_state.x = new_x;
  current_state.y = new_y;

  long new_a_steps = length_to_steps(new_a);
  long new_b_steps = length_to_steps(new_b);

  if (current_state.a_steps - new_a_steps == 0 && current_state.b_steps - new_b_steps == 0)
  {
    return;
  }

  a_dest = new_a_steps;
  b_dest = new_b_steps;

  if (log_debug)
  {
    Serial.print("Step dest: ");
    Serial.print(a_dest);
    Serial.print(" ");
    Serial.println(b_dest);
  }
  
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
//
//  if (log_debug)
//  {
//    Serial.print("Buffer: ");
//    Serial.print(buffer_front);
//    Serial.print(" ");
//    Serial.println(buffer_back);
//  }

 interrupts();
}

int lift_delay = 100;
int lift_position = 90;

void do_lift(bool lift)
{
  bool log_debug = false;

  if (log_debug)
  {
    Serial.print("Servo: ");
    Serial.println(lift ? lift_position : 0);
  }

  current_state.lift = lift;
  servo.moveToDegrees(lift ? lift_position : 0);
}

static char line[64];

void parse_line()
{ 
  uint8_t char_counter = 0;
  bool movement = false;

  gc_block current_block = buffer_last();
  
  while (line[char_counter] != 0)
  {
    if (line[char_counter] == ' ' || line[char_counter] == '\r' || line[char_counter] == '\n')
    {
      char_counter++;
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
        if (value == 0.0)
        {
          Serial.print(motor.getPositionSteps());
          Serial.print(" ");
     
          Serial.print(a_dest);
          Serial.print(" ");    
          Serial.print(a_speed);
          Serial.print(" ");  
          Serial.print(motor2.getPositionSteps());
          Serial.print(" ");
          Serial.print(b_dest);
          Serial.print(" "); 
          Serial.print(b_speed);
          Serial.print(" ");
          Serial.println(get_buffer_empty());
        }
        else
        {
//          Serial.print("parsed lift");
//          Serial.print(buffer_front);
//          Serial.print(" ");
//          Serial.print(current_block.x);
//          Serial.print(" ");
//          Serial.println(current_block.y);
          
          current_block.lift = value == 3;
        }
      }
      else if (letter == 'F')
      {
        current_state.feed = (int)value;
      }
      else if (letter == 'X')
      {
        current_block.x = value;
      }
      else if (letter == 'Y')
      {
        current_block.y = value;
      }
          
      continue;
    }
    else
    {
      Serial.println("Invalid number");
      return;
    }   

    char_counter++;
  }

  if (!get_buffer_full())
  {
    buffer_add(current_block);
    
    if (!get_buffer_full())
    {
      Serial.println("ok");
    }
  }
  else
  {
    Serial.println("dropped");
    
  }
}

void run_steppers()
{
  if (motor.getPositionSteps() == a_dest && motor2.getPositionSteps() == b_dest)
  {    
    if (!get_buffer_empty())
    {
      bool was_full = get_buffer_full();
      gc_block block = buffer_advance();

      if (block.x != current_state.x ||
          block.y != current_state.y)
      {
//          Serial.print("start move");
//          Serial.print(block.x);
//          Serial.print(block.y);
//          Serial.print(" ");
//          Serial.println(get_buffer_empty());
//          Serial.flush();            
         do_move(block.x, block.y);
      }

      if (block.lift != current_state.lift)
      {
        //Serial.println("start lift");
       // Serial.flush();
        
        do_lift(block.lift); 
        delay(1000);
      }

      if (was_full)
        Serial.println("ok");        
    }  

    
  }
}

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
          parse_line();
        
        char_counter = 0; 
        line[0] = 0;
      }
      else 
      {
        line[char_counter++] = c;
      }
    }
  
    run_steppers();

    Serial.flush();
  }
}

void stepper_isr()
{
  if (motor.getPositionSteps() != a_dest)
    motor.step();

  if (motor2.getPositionSteps() != b_dest)
    motor2.step();

   servo.update();
}

void setup()
{
 /*
  Serial.println(get_buffer_empty());
  Serial.println(buffer_add(gc_block(25.0, 25.0, true)));
  Serial.println(buffer_add(gc_block(-25.0, 25.0, true)));
  Serial.println(buffer_add(gc_block(-25.0, -25.0, true)));
  Serial.println(get_buffer_full());
  Serial.println(buffer_add(gc_block(25.0, -25.0, true)));
  Serial.println(get_buffer_full());
  Serial.println(buffer_add(gc_block(50.0, 50.0, true)));
  Serial.println(get_buffer_full());
  */

  Timer1.initialize(50);
  Timer1.attachInterrupt(stepper_isr);

  servo.moveToDegrees(0);
  delay(1000);
  servo.moveToDegrees(lift_position);

  motor.setSpeed(0);
  motor2.setSpeed(0);   

  Serial.begin(115200);
  Serial.setTimeout(0);
  
  while (!Serial)
    delay(10); // for micro  

  Serial.print("Ready\r\n");

  motor.setPosition(current_state.a_steps);
  motor2.setPosition(current_state.b_steps);

  Serial.print(current_state.a_steps);
  Serial.print(" ");
  Serial.println(current_state.b_steps);

  motor.enable();
  motor2.enable();
}

