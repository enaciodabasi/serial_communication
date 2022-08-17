#ifndef SERIALWORKER_H
#define SERIALWORKER_H  

#include <stdio.h>
#include <iostream>
#include <errno.h> // strerror
#include <unistd.h> // open, close, read, write functions
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <fcntl.h> // File Controls
#include <sys/time.h>
#include <termios.h> // Termios struct
#include <string>
#include <string.h> 
#include <vector>

class SerialWorker
{
    public:

    SerialWorker();
    SerialWorker(std::string serialPort, int baudrate);
    ~SerialWorker();

    private:

    std::string m_serialPort;
    int m_baudrate;
    int m_fd;
    struct termios m_tty;

    public:

    int InitSerialWorker();
    void CloseSerialWorker();

    
    char WriteChar(const char msg); // Send one char to the serial port
    char WriteString(const char* msg); // Send a string to the serial port
    char WriteBytes(const void *buffer, const unsigned int num_bytes); // Sent #num_bytes bytes to the serial port
    int WriteBinary(const uint8_t& msg);
    int WriteBinaryArray(const std::vector<uint8_t>& msg);
    

};

#endif