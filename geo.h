#pragma once

#include "config.h"
#include "Arduino.h"

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
