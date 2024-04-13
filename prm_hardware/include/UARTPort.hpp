#ifndef UARTPORT_HPP
#define UARTPORT_HPP
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <termios.h>
#include <cstdint>
#include <cmath>
#include <ctime>
#include <cstdlib>

class UARTPort
{
public:
    UARTPort(const std::string &port) : port_name(port), uart_fd(-1)
    {
        open_uart();
    }

    ~UARTPort()
    {
        close_uart();
    }

private:
    std::string port_name;
    int uart_fd;
    void open_uart();
    void close_uart();
    bool write_data(const uint8_t *data, size_t size);
};

#endif // UARTPORT_HPP
