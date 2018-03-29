#pragma once

#include "config.h"

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
