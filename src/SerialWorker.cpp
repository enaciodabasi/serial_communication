#include "../include/SerialWorker.h"

SerialWorker::SerialWorker()
{
    std::cout << "Set the Serial Port address and the Baud Rate" << std::endl;
}

SerialWorker::SerialWorker(std::string serialPort, int baudrate)
    : m_serialPort(serialPort), m_baudrate(baudrate)
{
    std::cout << "Serial Port address: " << m_serialPort << std::endl;
    std::cout << "Baud Rate: " << m_baudrate << std::endl;
}

SerialWorker::~SerialWorker()
{
    CloseSerialWorker();
}

int SerialWorker::InitSerialWorker()
{
    m_fd = open(m_serialPort.c_str(), O_RDWR | O_NOCTTY);

    if(tcgetattr(m_fd, &m_tty) != 0)
    {
        const int errnum = errno;
        fprintf(stderr, "%s: tcsetattr() failed: %s (%d)\n", m_serialPort.c_str(), strerror(errnum), errnum);
        return -1;
    }

    m_tty.c_cflag &= ~PARENB;
    m_tty.c_cflag &= ~CSTOPB;
    m_tty.c_cflag |= CS8;
    m_tty.c_cflag &= ~CRTSCTS;
    m_tty.c_cflag |= CREAD | CLOCAL;

    m_tty.c_lflag &= ~ICANON;
    m_tty.c_lflag &= ~ECHO; // Disable echo
    m_tty.c_lflag &= ~ECHOE; // Disable erasure
    m_tty.c_lflag &= ~ECHONL; // Disable new-line echo
    m_tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    m_tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    m_tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    m_tty.c_oflag &= ~OPOST;
    m_tty.c_oflag &= ~ONLCR;

    m_tty.c_cc[VTIME] = 10;
    m_tty.c_cc[VMIN] = 0;

    cfsetispeed(&m_tty, m_baudrate);
    cfsetospeed(&m_tty, m_baudrate);

    if(tcsetattr(m_fd, TCSANOW, &m_tty) != 0)
    {
        const int errnum = errno;
        fprintf(stderr, "%s: tcsetattr() failed: %s (%d)\n", m_serialPort.c_str(), strerror(errnum), errnum);
        return -1;
    }

    return m_fd;
}

void SerialWorker::CloseSerialWorker()
{
    close(m_fd);
    m_fd = -1;
}

char SerialWorker::WriteChar(const char msg)
{
    if(write(m_fd, &msg, sizeof(msg)) != 1)
    {
        return -1;
    }
    return 1;
}

char SerialWorker::WriteString(const char* msg)
{
    unsigned int msgLength = strlen(msg);

    if(write(m_fd, msg, msgLength) != msgLength)
    {
        return -1;
    }

    return 1;
}

char SerialWorker::WriteBytes(const void* buffer, const unsigned int num_bytes)
{
    if(write(m_fd, buffer, num_bytes) != (ssize_t)num_bytes)
    {
        return -1;
    }

    return 1;
}

int SerialWorker::WriteBinary(const uint8_t& msg)
{
    if(write(m_fd, &msg, sizeof(msg)) != 1)
    {
        return -1;
    }

    return 1;
}

int SerialWorker::WriteBinaryArray(const std::vector<uint8_t>& msg)
{
    if(write(m_fd, msg.data(), msg.size()) != 1) 
    {
        return -1;
    }

    return 1;
} 

