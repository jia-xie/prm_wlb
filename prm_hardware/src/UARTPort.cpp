#include "UARTPort.hpp"
#include <rclcpp/rclcpp.hpp>

void UARTPort::open_uart() {
    this->uart_fd = open(this->port_name.c_str(), O_RDWR);
    if (uart_fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open: %s, %s", port_name.c_str(), std::strerror(errno));
        return;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(uart_fd, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error from tcgetattr: %s", std::strerror(errno));
        return;
    }

    // Set Baud Rate
    cfsetospeed(&tty, B115200); // 115200 baud
    cfsetispeed(&tty, B115200);

    // Setting other Port Stuff
    // tty.c_cflag &= ~CSIZE; // Mask the character size bits
    // tty.c_cflag |= CS8;    // Set 8 data bits
    // tty.c_cflag |= PARENB; // Enable Parity - Note that this adds a parity bit, making it 9 bits in total
    // tty.c_cflag &= ~PARODD; // Set even parity

    tty.c_cflag &= ~CSIZE;  // Mask the character size bits
    tty.c_cflag |= CS8;     // Set 8 data bits
    tty.c_cflag &= ~PARENB; // Disable parity

    tty.c_cflag &= ~CSTOPB; // 1 Stop bit

    tty.c_cflag &= ~CRTSCTS;       // No flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                               // Disable echo
    tty.c_lflag &= ~ECHOE;                                              // Disable erasure
    tty.c_lflag &= ~ECHONL;                                             // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                               // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                             // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Fetch bytes as they become available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << std::strerror(errno) << std::endl;
    } else {
        std::cout << "UART Connected" << std::endl;
    }
}

// 0 for nothing, <0 for error, >0 for received
int UARTPort::read_data(char *buffer, size_t size)
{
    tcflush(uart_fd, TCIFLUSH);
    int received = read(uart_fd, buffer, size);
    if (received < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Read error: " << std::strerror(errno) << std::endl;
        }
        // Handle no data available
    } else if (received == 0) {
        // Handle no data received
    } else {
        // Data received
        std::cout << "Received " << received << " bytes." << std::endl;
    }
    return received;
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