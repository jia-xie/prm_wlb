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
#include "prm_interfaces/msg/unitree_motor.hpp"
#include "prm_interfaces/msg/remote_dr16_data.hpp"
#include "serialPort/SerialPort.h"
#include "UARTPort.hpp"
#include "unitreeMotor/unitreeMotor.h"
#include "RemoteDR16Driver.hpp"

class MotorCtrlNode : public rclcpp::Node
{
public:
    MotorCtrlNode(SerialPort *unitree_rs485_serial_port, UARTPort *uart_port);

    ~MotorCtrlNode();
private:
    RemoteDR16 remote_;
    prm_interfaces::msg::RemoteDR16Data remote_msg_;
    MotorCmd unitree_cmd;
    MotorData unitree_data;
    std::shared_ptr<SerialPort> unitree_rs485 = nullptr;
    std::shared_ptr<UARTPort> uart_port = nullptr;
    rclcpp::Subscription<prm_interfaces::msg::UnitreeMotor>::SharedPtr unitree_motor_subscriber;
    rclcpp::Publisher<prm_interfaces::msg::RemoteDR16Data>::SharedPtr remote_publisher_;
    rclcpp::TimerBase::SharedPtr lower_comm_timer_;
    void unitree_motor_callback(const prm_interfaces::msg::UnitreeMotor::SharedPtr msg);
    void lower_comm_timer_callback(void);
};

#endif
