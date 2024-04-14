from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Node to control the motor
    motor_ctrl_node = Node(
        package="prm_hardware",
        executable="LowerLevelCommNode",
        name="motor_ctrl_node"
    )

    # Node to control the chassis
    chassis_ctrl_node = Node(
        package="prm_control",
        executable="ChassisCtrlNode",
        name="chassis_ctrl_node"
    )

    ld.add_action(motor_ctrl_node)
    ld.add_action(chassis_ctrl_node)  # Uncomment this line to include the chassis control node in the launch

    return ld