#pragma once

// hack for std::optional support with Xcode 9.2
#ifdef WIN32
#include <optional>
template<typename N>
using opt = std::optional<N>;
#else
#include <experimental/optional>
template<typename N>
using opt = std::experimental::optional<N>;
#endif

using vec2 = std::pair<float, float>;
using pos2 = vec2;

const float PI = 3.1415927f;
const float TWO_PI = 2.0f * PI;
