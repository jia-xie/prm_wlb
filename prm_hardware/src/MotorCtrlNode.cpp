/**
 * @file MotorCtrlNode.cpp
 * @brief Motor Control Header.
 *
 * @author Jia Xie
 * @date March 31, 2024
 * @version 1.0
 *
 */
#include <unistd.h>
#include "MotorCtrlNode.hpp"
#include "prm_interfaces/msg/unitree_motor.hpp"


MotorCtrlNode::MotorCtrlNode(SerialPort *unitree_rs485_serial_port, UARTPort *uart_port) : 
            Node("motor_ctrl"), 
            unitree_rs485(unitree_rs485_serial_port),
            uart_port(uart_port)
{
    this->unitree_motor_subscriber = this->create_subscription<prm_interfaces::msg::UnitreeMotor>(
        "unitree_motor_cmd", 10, 
        std::bind(&MotorCtrlNode::unitree_motor_callback, this, std::placeholders::_1));
    RCLCPP_INFO_ONCE(this->get_logger(), "Motor Control Node Initialized");
}

MotorCtrlNode::~MotorCtrlNode()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Motor Control Node Terminated");
}

void MotorCtrlNode::unitree_motor_callback(const prm_interfaces::msg::UnitreeMotor::SharedPtr msg)
{
    unitree_cmd.motorType = MotorType::GO_M8010_6;
    unitree_data.motorType = MotorType::GO_M8010_6;
    unitree_cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    unitree_cmd.id = msg->id;
    unitree_cmd.kp = msg->kp;
    unitree_cmd.kd = msg->kd;
    unitree_cmd.q = msg->ang;
    unitree_cmd.dq = msg->vel * queryGearRatio(MotorType::GO_M8010_6);
    unitree_cmd.tau = msg->torq;
    this->unitree_rs485->sendRecv(&unitree_cmd, &unitree_data);

    RCLCPP_INFO(this->get_logger(), "motor.q: %f", unitree_data.q);
    RCLCPP_INFO(this->get_logger(), "motor.temp: %f", unitree_data.temp);
    RCLCPP_INFO(this->get_logger(), "motor.W: %f", unitree_data.dq);
    RCLCPP_INFO(this->get_logger(), "motor.merror: %f", unitree_data.merror);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    SerialPort *unitree_rs485_serial_port = new SerialPort("/dev/ttyUSB0");
    UARTPort *uart_port = new UARTPort("/dev/ttyUSB1");
    auto motor_ctrl_node = std::make_shared<MotorCtrlNode>(unitree_rs485_serial_port, uart_port);
    rclcpp::spin(motor_ctrl_node);
    rclcpp::shutdown();
}
