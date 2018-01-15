#pragma once

#include "block.h"

transformer in_to_mm()
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

transformer mm_to_in()
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

transformer center_x(const range & x_extent)
{
	auto x_center = (x_extent.second + x_extent.first) / 2.0f;

	return [x_center](block b)
	{
		if (b.x)
			b.x = *b.x - x_center;

		return b;
	};
}

transformer composite(std::vector<transformer> transforms)
{
	return [transforms](block b)
	{
		block tb = b;
		for (auto t : transforms)
			tb = t(tb);
		return tb;
	};
}
