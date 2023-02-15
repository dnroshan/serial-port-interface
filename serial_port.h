#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H

#include <termios.h>
#include <iostream>

class SerialPort {
public:
   SerialPort(std::string port);
   void release();
   void send(std::string str);
   std::string receive();

private:
   int fd;
   struct termios tty;
};

#endif /* _SERIAL_PORT_H */
