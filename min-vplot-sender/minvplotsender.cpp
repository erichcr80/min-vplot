#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#ifdef WIN32
#include "win\serial_windows.h"
#else
#include "serial_osx.h"
#endif

#include "parse.h"
#include "transforms.h"
#include "options.h"
#include "trace.h"

using namespace std;

/*
 minvplotsender

 This utility communicates with a min-vplot controller through a serial interface.
 It sends the contents of the provided NC file to the provided COM port/USB device.

 See options.h for arguments.
 */
int main(int argc, const char * argv[])
{
	auto opt = parse_options(argc, argv);

	if (opt.error)
	{
		cout << *opt.error << "\n\n" << opt.man << endl;
		return 1;
	}

	ifstream file(opt.nc_path, ifstream::in);

	if (!file)
	{
		cout << "Input file error:" << opt.nc_path << endl;
		return 1;
	}

	gcode_parser parser;

	string str;
	while (getline(file, str))
	{
		if (!parser.add(str))
		{
			cout << "NC file parsing error" << endl;
			return 1;
		}
	}

	list<block::transformer> transforms;

	if (opt.center_x)
		transforms.push_back(center_x(parser.get_x_extent()));

	if (opt.center_y)
		transforms.push_back(center_y(parser.get_y_extent()));

	if (opt.scale_width)
		transforms.push_back(scale_width(parser.get_x_extent(), *opt.scale_width));
	
	if (opt.scale_height)
		transforms.push_back(scale_height(parser.get_y_extent(), *opt.scale_height));

	if (opt.trace_extents_only)
	{
		gcode_parser extents_gcode;
		extents_gcode.add(make_outline_trace(parser.get_x_extent(), parser.get_y_extent()));

		parser = extents_gcode;
	}

	block::transformer all_transforms(composite(transforms));

//#define DUMP_DEBUG
#ifdef DUMP_DEBUG
	cout << "(x extent: (" << parser.get_x_extent().first << ", " << parser.get_x_extent().second << "), "
		<< "y extent: (" << parser.get_y_extent().first << ", " << parser.get_y_extent().second << "))" << endl;

	for (auto & block : parser)
	{
		std::cout << block.transform(all_transforms) << std::endl;
	}

	return 0;
#endif

#ifdef WIN32
	serial_win32 serial;
#else
	serial_osx serial;
#endif

	if (!serial.setup(opt.port_identifier))
	{
		return 1;
	}

	serial.sleep(100);
	serial.write(">");
	serial.sleep(100);

	string input;

	// Processing loop; await "Ready/ok" from controller and send lines when received.
	while (true)
	{
		const auto result = serial.read();

		if (!result)
		{
			return 1;
		}
		else if (result->length() > 0)
		{
			input.append(*result); // Add new input; may be incomplete response.

			auto line_ending_index = string::npos;

			while ((line_ending_index = input.find("\r\n")) != string::npos)
			{
				string line = input.substr(0, line_ending_index);
				cout << "<[" << line << "]" << endl;

				input = input.substr(line_ending_index + 2);

				if (line.compare("ok") == 0 || line.compare("Ready") == 0)
				{
					if (!parser.empty())
					{
						serial.write(parser.front().transform(all_transforms));

						parser.pop_front();
					}

					if (parser.empty()) // done
					{
						serial.write(block(pos2(0.0f, 0.0f))); // return to home
						return 0;
					}
				}
			}

			static bool debug_request_parameters = false;

			if (debug_request_parameters)
			{
				serial.write("M0V0.0");
				debug_request_parameters = false;
			}
		}

		serial.sleep(1 /* ms */);
	}

	return 0;
}

