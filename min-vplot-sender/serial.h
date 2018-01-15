#pragma once

#include "types.h"

#include <string>

class serial
{
public:
	virtual bool setup(const std::string & port_str) = 0;

	virtual optional<std::string> read() const = 0;
	virtual bool write(const std::string & string) const = 0;

	virtual void sleep(const unsigned int ms) const = 0;
};
