#ifndef CAN_PORT_HPP
#define CAN_PORT_HPP

#include <string>
#include <linux/can.h>


class CANPort {
public:
    CANPort(const std::string& interfaceName);
    ~CANPort();
    bool send(const can_frame& frame);

private:
    std::string mInterfaceName;
    int mSocket;
};

#endif  // CAN_PORT_HPP
