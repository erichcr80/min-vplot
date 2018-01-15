#pragma once

#include <string>
#include <ostream>
#include <sstream>

#include "types.h"

struct block;
std::ostream & operator<<(std::ostream & of, const block & b);

using transformer = std::function<block(block block)>;

struct block
{
	std::string line;

	units unit = units::unknown;

	static units new_block_unit;

	optional<int> g_number;
	optional<int> m_number;

	optional<float> x, y, i, j;

	block(pos2 position)
	{
		assert(new_block_unit != units::unknown);
		unit = new_block_unit;

		g_number = 1;

		x = position.first;
		y = position.second;
	}

	block(std::string _line)
	{
		line = _line;
		unit = new_block_unit;

		x = parse_float(line, 'X');
		y = parse_float(line, 'Y');
		i = parse_float(line, 'I');
		j = parse_float(line, 'J');

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

	operator std::string() const
	{
		std::stringstream buf;
		buf << *this;

		return buf.str();
	}

	static optional<float> parse_float(const std::string & line, char g_char)
	{
		const std::regex rr = std::regex("((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?");

		auto char_idx = line.find(g_char);

		if (char_idx == std::string::npos)
			return optional<float>();

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
