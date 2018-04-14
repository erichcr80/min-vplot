/* min-vplot: Minimal motion controller for v-plotter. */

#include "buffer.h"

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

int get_next()
{
  if (buffer_back >= BUFFER_SIZE - 1)
    return 0;
  else
    return buffer_back + 1;
}

bool get_buffer_full()
{
  return buffer_front == get_next();
}

gc_block buffer_advance()
{
  int current = buffer_front;
  buffer_front++;
  if (buffer_front >= BUFFER_SIZE)
    buffer_front = 0;
  return gc_buffer[current];
}

gc_block & buffer_current()
{
  return gc_buffer[buffer_front];
}

gc_block & buffer_last()
{
  return gc_buffer[buffer_back - 1 < 0 ? BUFFER_SIZE - 1 : buffer_back - 1];
}

bool buffer_add(gc_block gc_add)
{
  if (get_buffer_full())
    return false;

  int next = buffer_back;
  gc_buffer[next] = gc_add;

  buffer_back = get_next();

  return true;
}
