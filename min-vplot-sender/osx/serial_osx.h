#pragma once

#include "serial.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> // needed for memset

class serial_osx
{
    int tty_fd = 0;
    
public:
	virtual bool setup(const std::string & port_str)
    {
        struct termios tio;
        struct termios stdio;

        fd_set rdset;
        
        //printf("Please start with %s /dev/ttyS1 (for example)\n",argv[0]);
        memset(&stdio,0,sizeof(stdio));
        stdio.c_iflag=0;
        stdio.c_oflag=0;
        stdio.c_cflag=0;
        stdio.c_lflag=0;
        stdio.c_cc[VMIN]=1;
        stdio.c_cc[VTIME]=0;
        tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
        tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking
        
        memset(&tio,0,sizeof(tio));
        tio.c_iflag=0;
        tio.c_oflag=0;
        tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
        tio.c_lflag=0;
        tio.c_cc[VMIN]=1;
        tio.c_cc[VTIME]=5;
        
        tty_fd=::open(port_str.c_str(), O_RDWR);        // O_NONBLOCK might override VMIN and VTIME, so read() may return immediately.
        
        if (tty_fd == -1)
        {
            std::cout << "Error opening serial device: " << port_str << std::endl;
            return false;
        }
        
        cfsetospeed(&tio,B115200);            // 115200 baud
        cfsetispeed(&tio,B115200);            // 115200 baud
        
        tcsetattr(tty_fd,TCSANOW,&tio);
        
        return true;
    }

	virtual optional<std::string> read() const
    {
        unsigned int BUFFER_SIZE = 256;
        char buffer[BUFFER_SIZE];
        memset(buffer, 0, BUFFER_SIZE);
        
        if (::read(tty_fd, buffer, BUFFER_SIZE) >= 0)
        {
            return std::string(buffer);
        }
        else
        {
            return nullopt;
        }
    }
    
	virtual bool write(const std::string & string) const
    {
        std::string str_with_line_ending = string + "\r\n";

        if (!::write(tty_fd, str_with_line_ending.c_str(), str_with_line_ending.length()))
        {
            std::cout << "write_serial error" << std::endl;
            return false;
        }
        else
        {
            std::cout << ">[" << string << "]" << std::endl;
            return true;
        }
        
        ::tcdrain(tty_fd);
        
        return true;
    }
    
    virtual void sleep(const unsigned int ms) const
    {
        usleep(ms * 1000);
    }
};
