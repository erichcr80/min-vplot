#pragma once

#include "geo.h"
#include "config.h"

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
