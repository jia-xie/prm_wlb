/**
 * @file LowerLevelCommNode.cpp
 * @brief Motor Control Header.
 *
 * @author Jia Xie
 * @date March 31, 2024
 * @version 1.0
 *
 */
#include <unistd.h>
#include "LowerLevelCommNode.hpp"

LowerLevelCommNode::LowerLevelCommNode(SerialPort *unitree_rs485_serial_port, UARTPort *uart_port) : Node("motor_ctrl"),
                                                                                           unitree_rs485(unitree_rs485_serial_port),
                                                                                           uart_port(uart_port)
{
    this->unitree_motor_subscriber = this->create_subscription<prm_interfaces::msg::UnitreeMotor>(
        "unitree_motor_cmd", 10,
        std::bind(&LowerLevelCommNode::unitree_motor_callback, this, std::placeholders::_1));

    this->lower_comm_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), // Timer interval
        std::bind(&LowerLevelCommNode::lower_comm_timer_callback, this));
    this->remote_publisher_ = this->create_publisher<prm_interfaces::msg::RemoteDR16Data>("remote_dr16", 10);
    RCLCPP_INFO_ONCE(this->get_logger(), "Motor Control Node Initialized");
}

LowerLevelCommNode::~LowerLevelCommNode()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Motor Control Node Terminated");
}

void LowerLevelCommNode::unitree_motor_callback(const prm_interfaces::msg::UnitreeMotor::SharedPtr msg)
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

    // RCLCPP_INFO(this->get_logger(), "motor.q: %f", unitree_data.q);
    // RCLCPP_INFO(this->get_logger(), "motor.temp: %f", unitree_data.temp);
    // RCLCPP_INFO(this->get_logger(), "motor.W: %f", unitree_data.dq);
    // RCLCPP_INFO(this->get_logger(), "motor.merror: %f", unitree_data.merror);
}

void LowerLevelCommNode::lower_comm_timer_callback()
{
    constexpr int BUFFER_SIZE = 33 + 18;
    uint8_t buffer[BUFFER_SIZE];
    int byte_read = this->uart_port->read_data(reinterpret_cast<char *>(buffer), BUFFER_SIZE);
    std::string hex_output;

    if (byte_read > 0)
    {
        for (int i = 0; i < byte_read; i++)
        {
            char buf[4]; // Buffer to hold hex string for each byte
            sprintf(buf, "%02X ", buffer[i]);
            hex_output += buf;
        }
        if (byte_read == BUFFER_SIZE)
        {
            uint8_t remote_buffer[18];
            memcpy(remote_buffer, &buffer[33], 18);
            remote_.decode(remote_buffer);
            remote_.get_msg(this->remote_msg_);
            this->remote_publisher_->publish(remote_msg_);
            RCLCPP_INFO(this->get_logger(), "right x: %d", remote_.remote_.controller.right_stick.x);
            RCLCPP_INFO(this->get_logger(), "right x: %f", remote_msg_.right_joystick_x);
            RCLCPP_INFO(this->get_logger(), "%d byte: %s", byte_read, hex_output.c_str());
        }
        
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No data read from UART");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    SerialPort *unitree_rs485_serial_port = new SerialPort("/dev/ttyUSB0");
    UARTPort *uart_port = new UARTPort("/dev/ttyTHS0");
    auto lower_level_comm_node = std::make_shared<LowerLevelCommNode>(unitree_rs485_serial_port, uart_port);
    rclcpp::spin(lower_level_comm_node);
    rclcpp::shutdown();
}
