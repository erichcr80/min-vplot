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

using namespace std;

/*
 minvplotsender

 This utility communicates with a min-vplot controller through a serial interface.
 It sends the contents of the provided NC file to the provided COM port/USB device.

 Argument 1 - Serial port identifier (e.g. COM7, /dev/cu.usbmodem1411)
 Argument 2 - Path to NC file
 */
int main(int argc, const char * argv[])
{
#ifdef WIN32
	serial_win32 serial;
#else
	serial_osx serial;
#endif

	if (!serial.setup(argv[1]))
	{
		return 1;
	}

	ifstream file(argv[2], ifstream::in);
	vector<string> file_lines;

	if (!file)
	{
		cout << "Input file error" << endl;
		return 1;
	}

	// Read and store all file lines
	string str;
	while (getline(file, str))
		file_lines.push_back(str);

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
			input.append(*result); // Add new input; may be incomplete response

			auto line_ending_index = string::npos;

			while ((line_ending_index = input.find("\r\n")) != string::npos)
			{
				string line = input.substr(0, line_ending_index);
				cout << "<[" << line << "]" << endl;

				input = input.substr(line_ending_index + 2);

				if (line.compare("ok") == 0 || line.compare("Ready") == 0)
				{
					if (!file_lines.empty())
					{
						serial.write(file_lines.front());

						file_lines.erase(file_lines.begin(), file_lines.begin() + 1);
					}

					if (file_lines.empty()) // done
						return 0;
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

