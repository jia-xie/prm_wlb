#include "ChassisCtrlNode.hpp"

ChassisCtrlNode::ChassisCtrlNode() : Node("chassis_ctrl_node"),
left_state_(6),  // Initialize VectorXd of size 6
  right_state_(6), // Initialize VectorXd of size 6
  left_A_(6, 6),   // Initialize 6x6 MatrixXd
  right_A_(6, 6),  // Initialize 6x6 MatrixXd
  left_B_(6, 6),   // Initialize 6x6 MatrixXd
  right_B_(6, 6)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Chassis Control Node Initialization Stated");
    left_state_ << 0, 0, 0, 0, 0, 0;
    right_state_ << 0, 0, 0, 0, 0, 0;
    left_A_ << 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0,  
                0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0;

    left_motor_publisher_ = this->create_publisher<prm_interfaces::msg::UnitreeMotor>("unitree_motor_cmd", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2), // Timer interval
        std::bind(&ChassisCtrlNode::timer_callback, this));
    RCLCPP_INFO_ONCE(this->get_logger(), "Chassis Control Node Initialized");
}

ChassisCtrlNode::~ChassisCtrlNode()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Chassis Control Node Terminated");
}

void ChassisCtrlNode::timer_callback()
{
    auto unitree_cmd = prm_interfaces::msg::UnitreeMotor();
    unitree_cmd.id = 1;
    unitree_cmd.kp = 0.0;
    unitree_cmd.kd = 0.01;
    unitree_cmd.ang = 0.0;
    unitree_cmd.vel = -3.14;
    unitree_cmd.torq = 0.0;

    left_motor_publisher_->publish(unitree_cmd);
}

int main(int argc, char *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Chassis Control Node Started");
    rclcpp::init(argc, argv);
    auto chassis_ctrl_node = std::make_shared<ChassisCtrlNode>();
    rclcpp::spin(chassis_ctrl_node);
    rclcpp::shutdown();
}
