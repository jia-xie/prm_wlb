#include "UARTPort.hpp"

void UARTPort::open_uart() {
    this->uart_fd = open(this->port_name.c_str(), O_RDWR);
    if (uart_fd < 0) {
        std::cerr << "Failed to open: " << port_name << ", " << std::strerror(errno) << std::endl;
        return;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(uart_fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << std::strerror(errno) << std::endl;
        return;
    }
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << std::strerror(errno) << std::endl;
    } else {
        std::cout << "UART Connected" << std::endl;
    }
}

bool UARTPort::write_data(const uint8_t* data, size_t size) {
    if (write(uart_fd, data, size) != (ssize_t)size) {
        std::cerr << "Failed to write to port: " << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void UARTPort::close_uart() {
    if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
    }
}