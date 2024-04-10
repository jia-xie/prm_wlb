/**
 * @file MotorCtrlNode.hpp
 * @brief Motor Control Header.
 *
 * @author Jia Xie
 * @date March 31, 2024
 * @version 1.0
 *
 * Notes:
 */
#ifndef _MOTOR_CTRL_HPP
#define _MOTOR_CTRL_HPP

#include "rclcpp/rclcpp.hpp"

class MotorCtrlNode : public rclcpp::Node
{
public:
    MotorCtrlNode();

    ~MotorCtrlNode();
};

#endif
