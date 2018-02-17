#pragma once

#include "block.h"

block::transformer in_to_mm()
{
	return [](block b)
	{
		if (b.g_number && *b.g_number == 20)
			b.g_number = 21;

		if (b.unit == units::mm)
			return b;

		if (b.x)
			b.x = *b.x * 25.4f;

		if (b.y)
			b.y = *b.y * 25.4f;

		return b;
	};
}

block::transformer mm_to_in()
{
	return [](block b)
	{
		if (b.g_number && *b.g_number == 21)
			b.g_number = 20;

		if (b.unit == units::in)
			return b;

		if (b.x)
			b.x = *b.x / 25.4f;

		if (b.y)
			b.y = *b.y / 25.4f;

		return b;
	};
}

block::transformer center_x(const range & x_extent)
{
	auto x_center = (x_extent.second + x_extent.first) / 2.0f;

	return [x_center](block b)
	{
		if (b.x)
			b.x = *b.x - x_center;

		return b;
	};
}

block::transformer center_y(const range & y_extent)
{
	auto y_center = (y_extent.second + y_extent.first) / 2.0f;
	
	return [y_center](block b)
	{
		if (b.y)
			b.y = *b.y - y_center;
		
		return b;
	};
}

block::transformer scale_width(const range & original_x_extent, float new_width)
{
	auto original_width = original_x_extent.second - original_x_extent.first;
	auto scale_factor = new_width / original_width;
	
	return [scale_factor](block b)
	{
		if (b.x)
			b.x = *b.x * scale_factor;
		
		if (b.y)
			b.y = *b.y * scale_factor;
		
		return b;
	};
}

block::transformer scale_height(const range & original_y_extent, float new_height)
{
	auto original_height = original_y_extent.second - original_y_extent.first;
	auto scale_factor = new_height / original_height;
	
	return [scale_factor](block b)
	{
		if (b.x)
			b.x = *b.x * scale_factor;
		
		if (b.y)
			b.y = *b.y * scale_factor;
		
		return b;
	};
}

block::transformer composite(std::list<block::transformer> & transforms)
{
	return [transforms](block b)
	{
		block tb = b;
		for (auto t : transforms)
			tb = t(tb);
		return tb;
	};
}
