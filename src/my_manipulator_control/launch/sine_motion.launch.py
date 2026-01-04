import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # 패키지 경로
    description_pkg = get_package_share_directory('my_manipulator_description')
    
    # URDF 파일 경로
    urdf_file = os.path.join(description_pkg, 'urdf', 'my_robot.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # RViz 설정 파일
    rviz_config = os.path.join(description_pkg, 'rviz', 'view.rviz')
    
    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # sine_motion_publisher 노드
    sine_motion_node = Node(
        package='my_manipulator_control',
        executable='sine_motion',
        name='sine_motion_publisher',
        output='screen'
    )
    
    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        sine_motion_node,
        rviz_node
    ])
