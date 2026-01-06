from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('my_manipulator_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_robot.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'view.rviz'])
    
    # robot_description을 Command로 xacro 처리
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    nodes = [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{
                'zeros': {
                    'joint1': 1.5708,  # 90도 (라디안)
                    'joint2': 1.5708,  # 90도
                    'joint3': 0.0,     # 0도
                    'joint4': 3.1416,  # 180도
                    'gripper_left_joint_1': 3.1416,  # 180도
                }
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', rviz_config],
            output="screen",
        ),
    ]

    return LaunchDescription(nodes)