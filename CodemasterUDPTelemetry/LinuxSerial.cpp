#include "LinuxSerial.h"
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/file.h>

Serial::Serial(const char *portName)
{
	fd = open(portName, O_RDWR);
	if (fd < 0) {
		std::cerr << "Error " << errno << " from open: " << strerror(errno) << std::endl;
		throw std::runtime_error("Cannot open serial port");
	}

	if(flock(fd, LOCK_EX | LOCK_NB) == -1) {
		throw std::runtime_error("Serial port with file descriptor " + 
			std::to_string(fd) + " is already locked by another process.");
	}

	struct termios tty;
	memset(&tty, 0, sizeof tty);

	if (tcgetattr(fd, &tty) != 0) {
		std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
		throw std::runtime_error("Cannot get serial port properties");
	}

	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
		throw std::runtime_error("Cannot set serial port properties");
	}
}

Serial::~Serial()
{
	if (fd > -1)
		close(fd);
}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
	return read(fd, buffer, nbChar);
}

bool Serial::WriteData(const char *buffer, size_t nbChar)
{
	return write(fd, buffer, nbChar) == nbChar;
}

//Check if we are actually connected
bool Serial::IsConnected()
{
	return fd >= 0;
}
