/* min-vplot: Minimal motion controller for v-plotter. */

#include "Arduino.h"

#include "parse.h"
#include "config.h"
#include "gcode.h"
#include "buffer.h"
#include "machine.h"

#include "grbl_read_float.h" // from grbl

void parse_line(machine_state & current_state)
{
  uint8_t char_counter = 0;
  bool movement = false;
  bool comment = false;

  gc_block current_block = buffer_last();

  while (line[char_counter] != 0)
  {
    char ch = line[char_counter];

    if (ch == ' ' || ch == '\r' || ch == '\n')
    {
      char_counter++;
      continue;
    }

    if (ch == '(' || comment)
    {
      comment = true;
      char_counter++;
      continue;
    }

    if (ch == ')')
    {
      comment = false;
      char_counter++;
      continue;
    }

    char_counter++; // TODO is this needed?

    float value = 0.0;
    if (read_float(line, &char_counter, &value))
    {
      if (value > 1e9 || value < -1e9 || value == NAN)
      {
        Serial.print("Parsing error: ");
        Serial.println(value);
      }

      if (ch == 'G')
      {
        current_state.rapid = value == 1;
      }
      else if (ch == 'M')
      {
        if (value == 0.0) /* diagnostic printout */
        {
          Serial.print(current_state.a_dest);
          Serial.print(" ");
          Serial.print(current_state.motor_a.getSpeed());
          Serial.print(" ");
          Serial.print(current_state.b_dest);
          Serial.print(" ");
          Serial.print(current_state.motor_b.getSpeed());
          Serial.print(" ");
          Serial.print(current_state.motor_a.getPositionSteps());
          Serial.print(" ");
          Serial.println(current_state.motor_b.getPositionSteps());
        }
        else
        {
          current_block.lift = value == 3;
        }
      }
      else if (ch == 'F')
      {
        current_block.feed = min((int)value, MAX_FEED);
      }
      else if (ch == 'X')
      {
        current_block.pt.x = value;
      }
      else if (ch == 'Y')
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
