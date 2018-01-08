#pragma once

#include <string>
#include <sstream>
#include <regex>

#include "types.h"
#include "arc.h"

class gcode_parser : public std::vector<std::string>
{
    float x = 0.0, y = 0.0;
    
    void update_pos(opt<float> x_value, opt<float> y_value)
    {
        if (x_value)
            x = *x_value;
        
        if (y_value)
            y = *y_value;
    }
    
    static opt<float> parse_float(const std::string line, char g_char)
    {
        const std::regex rr = std::regex("((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?");
        
        auto char_idx = line.find(g_char);
        
        if (char_idx == std::string::npos)
            return opt<float>();
        
        std::smatch match;
        const std::string match_str = line.substr(char_idx + 1);
        if (std::regex_search(match_str, match, rr))
            return std::stof(match.str(0));
        
        return opt<float>();
    }
    
public:
    bool add(const std::string line)
    {
        auto add_move = [&](pos2 loc)
        {
            std::stringstream buf;
            
            buf << "G01 ";
            buf << "X" << loc.first << " ";
            buf << "Y" << loc.second << " ";
            
            push_back(buf.str());
        };
        
        if (line.find("G02") != std::string::npos || line.find("G03") != std::string::npos)
        {
            const auto x_value = parse_float(line, 'X');
            const auto y_value = parse_float(line, 'Y');
            const auto i_value = parse_float(line, 'I');
            const auto j_value = parse_float(line, 'J');
            
            bool is_g2 = line.find("G02") != std::string::npos;
            
            if (!move_arc(pos2(x, y), pos2(*x_value, *y_value), pos2(*i_value, *j_value), 0.5 /* tol - mm */, is_g2 ? cw : ccw, add_move))
                return false;
            
            update_pos(x_value, y_value);
        }
        else if (line.find("G00") != std::string::npos || line.find("G01") != std::string::npos)
        {
            const auto x_value = parse_float(line, 'X');
            const auto y_value = parse_float(line, 'Y');
            
            update_pos(x_value, y_value);
            
            add_move(pos2(x, y));
        }
        else
        {
            push_back(line);
        }
                
        return true;
    }
};
