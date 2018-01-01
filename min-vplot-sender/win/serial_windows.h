#pragma once

#include "serial.h"

#include <Windows.h>

#define buf_size 1024

class serial_win32 : public serial
{
	HANDLE m_handle = 0;

public:
	virtual bool setup(const std::string & port_str)
	{
		std::wstring wport_str(port_str.length(), L' ');
		copy(port_str.begin(), port_str.end(), wport_str.begin());

		m_handle = CreateFile(wport_str.c_str(),
			GENERIC_READ | GENERIC_WRITE,
			0,
			0,
			OPEN_EXISTING,
			0,
			0);

		if (m_handle == INVALID_HANDLE_VALUE)
		{
			std::cout << "COM port error" << std::endl;
			return false;
		}

		DCB dcb;

		FillMemory(&dcb, sizeof(dcb), 0);
		if (!GetCommState(m_handle, &dcb))
			std::cout << "comm error" << std::endl;

		dcb.BaudRate = CBR_115200;
		dcb.StopBits = ONESTOPBIT;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;

		if (!SetCommState(m_handle, &dcb))
		{
			std::cout << "state error" << std::endl;
			return false;
		}

		COMMTIMEOUTS times;
		times.ReadIntervalTimeout = MAXDWORD;
		times.ReadTotalTimeoutMultiplier = MAXDWORD;
		times.ReadTotalTimeoutConstant = 100;
		times.WriteTotalTimeoutMultiplier = 0;
		times.WriteTotalTimeoutConstant = 0;

		if (!SetCommTimeouts(m_handle, &times))
		{
			std::cout << "timeout error" << std::endl;
			return false;
		}

		return true;
	}

	virtual std::optional<std::string> read() const
	{
		DWORD chars_read = 0;

		char buf[buf_size];
		memset(buf, 0, buf_size);

		std::string input;

		if (!ReadFile(m_handle, buf, buf_size, &chars_read, NULL))
			return nullptr;

		input.append(buf);
		return input;
	}

	virtual bool write(const std::string & string) const
	{
		std::string str_with_line_ending = string + "\r\n";

		DWORD bytes_written = 0;
		if (!WriteFile(m_handle, str_with_line_ending.c_str(), static_cast<DWORD>(str_with_line_ending.length()), &bytes_written, NULL))
		{
			std::cout << "write_serial error" << std::endl;
			return false;
		}
		else
		{
			std::cout << ">[" << string << "]" << std::endl;
			return true;
		}
	}

	virtual void sleep(const unsigned int ms) const
	{
		Sleep(ms);
	}
};