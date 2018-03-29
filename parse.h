#pragma once

#include "grbl_read_float.h" // from grbl

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
