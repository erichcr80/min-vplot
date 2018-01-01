#pragma once

#include <string>

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

class serial
{
public:
	virtual bool setup(const std::string & port_str) = 0;

	virtual opt<std::string> read() const = 0;
	virtual bool write(const std::string & string) const = 0;

	virtual void sleep(const unsigned int ms) const = 0;
};
