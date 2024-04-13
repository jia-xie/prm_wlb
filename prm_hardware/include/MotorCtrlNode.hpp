/**
 * @file MotorCtrlNode.hpp
 * @brief Motor Control Header.
 *
 * @author Jia Xie
 * @date March 31, 2024
 * @version 1.0
 *
 */
#ifndef _MOTOR_CTRL_HPP
#define _MOTOR_CTRL_HPP

#include "rclcpp/rclcpp.hpp"
#include "prm_interfaces/msg/state_machine.hpp"
#include "prm_interfaces/msg/unitree_motor.hpp"
#include "serialPort/SerialPort.h"
#include "UARTPort.hpp"
#include "unitreeMotor/unitreeMotor.h"

class MotorCtrlNode : public rclcpp::Node
{
public:
    MotorCtrlNode(SerialPort *unitree_rs485_serial_port, UARTPort *uart_port);

    ~MotorCtrlNode();
private:
    MotorCmd unitree_cmd;
    MotorData unitree_data;
    std::shared_ptr<SerialPort> unitree_rs485 = nullptr;
    std::shared_ptr<UARTPort> uart_port = nullptr;
    std::shared_ptr<rclcpp::Subscription<prm_interfaces::msg::UnitreeMotor>> unitree_motor_subscriber;
    void unitree_motor_callback(const prm_interfaces::msg::UnitreeMotor::SharedPtr msg);
};

#endif
