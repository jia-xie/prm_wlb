#include "driver/CANPort.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include "rclcpp/rclcpp.hpp"

CANPort::CANPort(const std::string &interfaceName) : mInterfaceName(interfaceName), mSocket(-1)
{
    // Create a socket
    mSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mSocket < 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to create socket");
        return;
    }

    // Get the interface index
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, mInterfaceName.c_str(), IFNAMSIZ - 1);
    if (ioctl(mSocket, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to get interface index");
        close(mSocket);
        mSocket = -1;
        return;
    }

    // Bind the socket to the interface
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(mSocket, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to bind socket to interface");
        close(mSocket);
        mSocket = -1;
        return;
    }
}

CANPort::~CANPort()
{
    if (mSocket >= 0)
    {
        close(mSocket);
    }
}

bool CANPort::send(const can_frame &frame)
{
    if (mSocket < 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket is not initialized");
        return false;
    }

    ssize_t nbytes = write(mSocket, &frame, sizeof(frame));
    if (nbytes < 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to send CAN frame");
        return false;
    }

    return true;
}