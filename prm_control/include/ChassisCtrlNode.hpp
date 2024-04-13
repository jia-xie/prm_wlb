#ifndef CHASSIS_CTRL_NODE_HPP
#define CHASSIS_CTRL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "prm_interfaces/msg/unitree_motor.hpp"

class ChassisCtrlNode : public rclcpp::Node
{
public:
    ChassisCtrlNode();
    ~ChassisCtrlNode();
private:
    Eigen::VectorXd left_state_;
    Eigen::VectorXd right_state_;
    Eigen::MatrixXd left_A_;
    Eigen::MatrixXd right_A_;
    Eigen::MatrixXd left_B_;
    Eigen::MatrixXd right_B_;

    rclcpp::Publisher<prm_interfaces::msg::UnitreeMotor>::SharedPtr left_motor_publisher_;
    rclcpp::TimerBase::SharedPtr timer_; // Timer object

    void timer_callback();
};

#endif  // CHASSIS_CTRL_NODE_HPP