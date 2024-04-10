#include "MotorCtrlNode.hpp"

MotorCtrlNode::MotorCtrlNode() : Node("motor_ctrl_node")
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Motor Control Node Initialized");
}

MotorCtrlNode::~MotorCtrlNode()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Motor Control Node Terminated");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto motor_ctrl_node = std::make_shared<MotorCtrlNode>();
    rclcpp::spin(motor_ctrl_node);
    rclcpp::shutdown();
}
