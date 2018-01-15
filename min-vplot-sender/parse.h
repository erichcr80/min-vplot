#pragma once

#include <string>
#include <sstream>
#include <regex>

#include "types.h"
#include "arc.h"
#include "block.h"

class gcode_parser : public std::vector<block>
{
	range x_extent = range(1e6f, -1e6f), y_extent = range(1e6f, -1e6f);
	float x = 0.0f, y = 0.0f;

	void update_pos(optional<float> x_value, optional<float> y_value)
	{
		auto extend_range = [](range & r, float val)
		{
			if (val < r.first)
				r.first = val;
			else if (val > r.second)
				r.second = val;
		};

		if (x_value)
		{
			x = *x_value;
			extend_range(x_extent, x);
		}

		if (y_value)
		{
			y = *y_value;
			extend_range(y_extent, y);
		}
	}

public:
	range get_x_extent() const { return x_extent; }
	range get_y_extent() const { return y_extent; }

	bool add(const std::string & line)
	{
		block b(line);

		if (b.g_number && (*b.g_number == 2 || *b.g_number == 3))
		{
			auto arc_blocks = move_arc(
				pos2(x, y),
				pos2(*b.x, *b.y),
				pos2(*b.i, *b.j),
				0.01f /* tol - mm */,
				*b.g_number == 2 ? cw : ccw);

			insert(end(), arc_blocks.begin(), arc_blocks.end());

			update_pos(*b.x, *b.y);
		}
		else if (b.g_number && (*b.g_number == 0 || *b.g_number == 1))
		{
			update_pos(*b.x, *b.y);

			push_back(b);
		}
		else if (b.g_number && (*b.g_number == 20 || *b.g_number == 21))
		{
			block::new_block_unit = *b.g_number == 20 ? units::in : units::mm;

			push_back(b);
		}
		else
		{
			push_back(b);
		}

		return true;
	}
};
