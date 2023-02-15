#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/file.h>
#include "serial_port.h"

SerialPort::SerialPort(std::string port)
{
   fd = open(port.c_str(), O_RDWR);
   if (fd < 0) {
      fprintf(stderr, "Error %i from open: %s\n", errno, strerror(errno));
      exit(errno);
   }

   if (tcgetattr(fd, &tty) != 0) {
      fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
      exit(errno);
   }

   tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
   tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
   tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
   tty.c_cflag |= CS8; // 8 bits per byte (most common)
   tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
   tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

   tty.c_lflag &= ~ICANON;
   tty.c_lflag &= ~ECHO; // Disable echo
   tty.c_lflag &= ~ECHOE; // Disable erasure
   tty.c_lflag &= ~ECHONL; // Disable new-line echo
   tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
   tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
   tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

   tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
   tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
   // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
   // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

   tty.c_cc[VTIME] = 15;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
   tty.c_cc[VMIN] = 0;

   cfsetispeed(&tty, B115200);
   cfsetospeed(&tty, B115200);

   if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
      exit(errno);
   }
}

void SerialPort::release()
{
   close(fd);
}

void SerialPort::send(std::string str)
{
   write(fd, str.c_str(), str.length());
}

std::string SerialPort::receive()
{
   char buffer[1024];
   char c;
   int i = 0, n;
   while ((n = read(fd, &c, 1)) > 0 && c != '\n' && i < 1024) {
      buffer[i++] = c;
   }

   buffer[i] = '\0';

   return std::string(buffer);
}
