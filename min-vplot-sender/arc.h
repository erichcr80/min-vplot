#pragma once

#include "types.h"

#include <functional>

#include <assert.h>
#include <cmath>

enum move_arc_dir
{
	cw,
	ccw
};

bool move_arc(pos2 start, pos2 dest, pos2 center, float arc_tol, move_arc_dir dir, std::function<void(pos2)> add_move)
{
	const vec2 start_vec{ start.first - center.first, start.second - center.second };
	const vec2 dest_vec{ dest.first - center.first, dest.second - center.second };

	const float start_angle = atan2(start_vec.second, start_vec.first);
	const float dest_angle = atan2(dest_vec.second, dest_vec.first);

	const float arc_angle = dir == ccw ?
		(dest_angle - start_angle) :
		(TWO_PI - dest_angle - start_angle);

	if (arc_angle < 0.0)
	{
		assert(!"G2/3 arc angle invalid");
		return false;
	}

	const float arc_radius = sqrt(pow(start_vec.first, 2) + pow(start_vec.second, 2));
	const float dest_arc_radius = sqrt(pow(dest_vec.first, 2) + pow(dest_vec.second, 2));
	if (abs(arc_radius - dest_arc_radius) > 0.1)
	{
		assert(!"start/destination radii different");
		return false;
	}

	const float arc_length = arc_angle * arc_radius;

	if (arc_length < arc_tol) /* Arc length less than arc tolerance, just move to destination. */
	{
		add_move(dest);
		return true;
	}

	/* This is the angle corresponding to the arc tolerance. Step through the arc by this amount. */
	const float tol_angle = arc_angle * arc_tol / arc_length * (dir == cw ? -1.0f : 1.0f);

	float current_angle = fmod(start_angle + tol_angle + TWO_PI, TWO_PI);
	while (dir == ccw ? current_angle < dest_angle : current_angle > dest_angle)
	{
		const float move_x = cos(current_angle) * arc_radius + center.first;
		const float move_y = sin(current_angle) * arc_radius + center.second;

		add_move(pos2(move_x, move_y));

		current_angle += tol_angle;
	}

	add_move(dest);
	return true;
}
