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
#include "prm_interfaces/msg/remote_dr16.hpp"

MotorCtrlNode::MotorCtrlNode(SerialPort *unitree_rs485_serial_port, UARTPort *uart_port) : Node("motor_ctrl"),
                                                                                           unitree_rs485(unitree_rs485_serial_port),
                                                                                           uart_port(uart_port)
{
    this->unitree_motor_subscriber = this->create_subscription<prm_interfaces::msg::UnitreeMotor>(
        "unitree_motor_cmd", 10,
        std::bind(&MotorCtrlNode::unitree_motor_callback, this, std::placeholders::_1));

    this->lower_comm_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // Timer interval
        std::bind(&MotorCtrlNode::lower_comm_timer_callback, this));
    this->remote_publisher_ = this->create_publisher<prm_interfaces::msg::RemoteDR16>("remote_dr16", 10);
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

    // RCLCPP_INFO(this->get_logger(), "motor.q: %f", unitree_data.q);
    // RCLCPP_INFO(this->get_logger(), "motor.temp: %f", unitree_data.temp);
    // RCLCPP_INFO(this->get_logger(), "motor.W: %f", unitree_data.dq);
    // RCLCPP_INFO(this->get_logger(), "motor.merror: %f", unitree_data.merror);
}

void MotorCtrlNode::lower_comm_timer_callback()
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
            auto remote_msg = prm_interfaces::msg::RemoteDR16();
            remote_msg.right_joystick_x = ((buffer[0] | (buffer[1] << 8)) & 0x07ff) - 1024;
            remote_msg.right_joystick_y = (((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07ff) - 1024;
            remote_msg.left_joystick_x = (((buffer[2] >> 6) | (buffer[3] << 2) | (buffer[4] << 10)) & 0x07ff) - 1024;
            remote_msg.left_joystick_y = (((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07ff) - 1024;
            remote_msg.dial_wheel = ((buffer[16] | (buffer[17] << 8)) & 0x07FF) - 1024;
            remote_msg.left_switch = ((buffer[5] >> 4) & 0x000C) >> 2;
            remote_msg.right_switch = ((buffer[5] >> 4) & 0x0003);
            RCLCPP_INFO(this->get_logger(), "right_joystick_x: %d", remote_msg.right_joystick_x);

            // mouse decode
            remote_msg.mouse_x = (buffer[6]) | (buffer[7] << 8);
            remote_msg.mouse_y = buffer[8] | (buffer[9] << 8);
            remote_msg.mouse_z = buffer[10] | (buffer[11] << 8);
            remote_msg.mouse_left = buffer[12];
            remote_msg.mouse_right = buffer[13];

            // key decode
            uint16_t key_buffer = buffer[14] | (buffer[15] << 8);
            remote_msg.pressed_w = (key_buffer >> 0) & 0x001;
            remote_msg.pressed_s = (key_buffer >> 1) & 0x001;
            remote_msg.pressed_a = (key_buffer >> 2) & 0x001;
            remote_msg.pressed_d = (key_buffer >> 3) & 0x001;
            remote_msg.pressed_shift = (key_buffer >> 4) & 0x001;
            remote_msg.pressed_ctrl = (key_buffer >> 5) & 0x001;
            remote_msg.pressed_q = (key_buffer >> 6) & 0x001;
            remote_msg.pressed_e = (key_buffer >> 7) & 0x001;
            remote_msg.pressed_r = (key_buffer >> 8) & 0x001;
            remote_msg.pressed_f = (key_buffer >> 9) & 0x001;
            remote_msg.pressed_g = (key_buffer >> 10) & 0x001;
            remote_msg.pressed_z = (key_buffer >> 11) & 0x001;
            remote_msg.pressed_x = (key_buffer >> 12) & 0x001;
            remote_msg.pressed_c = (key_buffer >> 13) & 0x001;
            remote_msg.pressed_v = (key_buffer >> 14) & 0x001;
            remote_msg.pressed_b = (key_buffer >> 15) & 0x001;
            this->remote_publisher_->publish(remote_msg);
            RCLCPP_INFO(this->get_logger(), "Data in HEX: %s", hex_output.c_str());
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
    auto motor_ctrl_node = std::make_shared<MotorCtrlNode>(unitree_rs485_serial_port, uart_port);
    rclcpp::spin(motor_ctrl_node);
    rclcpp::shutdown();
}
