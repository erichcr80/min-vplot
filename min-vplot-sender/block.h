#pragma once

#include <string>
#include <ostream>
#include <sstream>

#include "types.h"

struct block;
std::ostream & operator<<(std::ostream & of, const block & b);

struct block
{
	using transformer = std::function<block(block block)>;
    
	std::string line;

	units unit = units::unknown;

	static units new_block_unit;

	optional<int> g_number;
	optional<int> m_number;

	optional<float> x, y, i, j;

	block(pos2 position, units unit) : unit(unit)
	{
		g_number = 1;

		x = get_converted(position.first);
		y = get_converted(position.second);
		
		unit = units::mm;
	}

	block(std::string _line)
	{
		line = _line;
		unit = new_block_unit;

		x = parse_float(line, 'X');
		y = parse_float(line, 'Y');
		i = parse_float(line, 'I');
		j = parse_float(line, 'J');
		
		if (x)
			x = get_converted(*x);

		if (y)
			y = get_converted(*y);
		
		if (i)
			i = get_converted(*i);
		
		if (j)
			j = get_converted(*j);
		
		unit = units::mm;
		
		g_number = parse_int(line, 'G');
		m_number = parse_int(line, 'M');
	}

	bool parsed() const
	{
		return x || y || i || j;
	}

	block transform(transformer t)
	{
		return t(*this);
	}
	
	float get_converted(float value)
	{
		if (unit == units::in)
			return value * 25.4f;
		
		return value;
	}
	
	operator std::string() const
	{
		std::stringstream buf;
		buf << *this;

		return buf.str();
	}

	static optional<float> parse_float(const std::string & line, char g_char)
	{
		auto char_idx = line.find(g_char);

		if (char_idx == std::string::npos)
			return optional<float>();

        const std::regex rr = std::regex("((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?");

		std::smatch match;
		const std::string match_str = line.substr(char_idx + 1);
		if (std::regex_search(match_str, match, rr))
			return std::stof(match.str(0));

		return optional<float>();
	}

	static optional<int> parse_int(const std::string & line, char g_char)
	{
		auto fres = parse_float(line, g_char);

		if (fres)
			return optional<int>(static_cast<int>(*fres));

		return nullopt;
	}
};

std::ostream & operator<<(std::ostream & of, const block & b)
{
	std::stringstream buf;
	buf.precision(4);

	if (!b.parsed())
	{
		of << b.line;
		return of;
	}

	if (b.g_number)
		buf << "G" << *b.g_number << " ";

	if (b.m_number)
		buf << "M" << *b.m_number << " ";

	if (b.x)
		buf << "X" << *b.x << " ";

	if (b.y)
		buf << "Y" << *b.y << " ";

	of << buf.str();
	return of;
};

// static
units block::new_block_unit = units::unknown;
