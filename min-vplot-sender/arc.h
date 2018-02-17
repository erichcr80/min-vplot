#pragma once

#include <functional>

#include <assert.h>
#include <cmath>

#include "types.h"
#include "block.h"

enum move_arc_dir
{
	cw,
	ccw
};

std::vector<block> move_arc(pos2 start, pos2 dest, pos2 dcenter, float arc_tol, move_arc_dir dir)
{
	std::vector<block> blocks;

	const vec2 center{ start.first + dcenter.first, start.second + dcenter.second };

	const vec2 start_vec{ -dcenter.first, -dcenter.second };
	const vec2 dest_vec{ dest.first - center.first, dest.second - center.second };

	const float start_angle = fmod(TWO_PI + atan2(start_vec.second, start_vec.first), TWO_PI);
	const float dest_angle = fmod(TWO_PI + atan2(dest_vec.second, dest_vec.first), TWO_PI);

	const float arc_angle_original = dir == ccw ? dest_angle - start_angle : start_angle - dest_angle;
	const float arc_angle = fmod(TWO_PI + arc_angle_original, TWO_PI);

	if (arc_angle < 0.0)
	{
		assert(!"G2/3 arc angle invalid");
		return blocks;
	}

	const float arc_radius = sqrt(pow(start_vec.first, 2) + pow(start_vec.second, 2));
	const float dest_arc_radius = sqrt(pow(dest_vec.first, 2) + pow(dest_vec.second, 2));
	if (abs(arc_radius - dest_arc_radius) > 0.50)
	{
		assert(!"start/destination radii different");
		return blocks;
	}

	const float arc_length = arc_angle * arc_radius;
	assert(arc_length > 0.0);

	if (arc_length < arc_tol) /* Arc length less than arc tolerance, just move to destination. */
	{
		blocks.push_back(block(dest, units::mm));
		return blocks;
	}

	/* This is the angle corresponding to the arc tolerance. Step through the arc by this amount. */
	const float tol_angle = arc_angle * arc_tol / arc_length;

	float current_dangle = tol_angle;
	while (current_dangle < arc_angle)
	{
		const float current_angle = (dir == cw ? -1.0f : 1.0f) * current_dangle + start_angle;

		const float move_x = cos(current_angle) * arc_radius + center.first;
		const float move_y = sin(current_angle) * arc_radius + center.second;

		blocks.push_back(block(pos2(move_x, move_y), units::mm));

		current_dangle += tol_angle;
	}

	blocks.push_back(block(dest, units::mm));
	return blocks;
}
