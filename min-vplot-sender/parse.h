#pragma once

#include <list>
#include <string>
#include <sstream>
#include <regex>

#include "types.h"
#include "arc.h"
#include "block.h"

class gcode_parser : public std::list<block>
{
	range x_extent = range(1e6f, -1e6f);
	range y_extent = range(1e6f, -1e6f);
	
	float x = 0.0f;
	float y = 0.0f;
    
	void update_pos(block & b)
	{
		auto extend_range = [](range & r, float val)
		{
			if (val < r.first)
				r.first = val;
			else if (val > r.second)
				r.second = val;
		};

		if (b.x)
		{
			x = *b.x;
			extend_range(x_extent, x);
		}

		if (b.y)
		{
			y = *b.y;
			extend_range(y_extent, y);
		}
	}

public:
	range get_x_extent() const { return x_extent; }
	range get_y_extent() const { return y_extent; }

	bool add(const std::string & line)
	{
		if (line.empty())
			return true;
		
		const auto cr_idx = line.find('\r');
		const auto line_trimmed = line.substr(0, cr_idx != std::string::npos ? cr_idx : line.length());
		
		if (line_trimmed.length() == 0)
			return true;
	
		block b(line_trimmed);
		
		if (b.g_number && (*b.g_number == 2 || *b.g_number == 3))
		{
			auto arc_blocks = move_arc(
				pos2(x, y),
				pos2(*b.x, *b.y),
				pos2(*b.i, *b.j),
				0.5f /* tol - mm */,
				*b.g_number == 2 ? cw : ccw);
			
			insert(end(), arc_blocks.begin(), arc_blocks.end());
			
			update_pos(b);
		}
		else if (b.g_number && (*b.g_number == 0 || *b.g_number == 1))
		{
			update_pos(b);
			
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
