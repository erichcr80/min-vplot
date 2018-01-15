#pragma once

// hack for std::optional support with Xcode 9.2
#ifdef WIN32
#include <optional>
using std::optional;
using std::nullopt;
#else
#include <experimental/optional>
using std::experimental::optional;
using std::experimental::nullopt;
#endif

using vec2 = std::pair<float, float>;
using pos2 = vec2;

using range = std::pair<float, float>;

const float PI = 3.1415927f;
const float TWO_PI = 2.0f * PI;

enum class units
{
	unknown,
	in,
	mm
};
