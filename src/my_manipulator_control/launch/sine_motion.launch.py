import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
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
    
    # Launch Argument: mode (virtual 또는 real)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='virtual',
        description='Control mode: virtual (simulation) or real (hardware)'
    )
    
    # Launch Argument: serial port
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for UART communication'
    )
    
    # Launch Argument: baudrate
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for UART communication'
    )
    
    mode = LaunchConfiguration('mode')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    
    # robot_state_publisher 노드 (항상 실행)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # sine_motion_publisher 노드 (항상 실행)
    sine_motion_node = Node(
        package='my_manipulator_control',
        executable='sine_motion',
        name='sine_motion_publisher',
        output='screen'
    )
    
    # UART controller 노드 (real 모드일 때만 실행)
    uart_controller_node = Node(
        package='my_manipulator_control',
        executable='uart_controller',
        name='uart_controller',
        parameters=[{
            'port': port,
            'baudrate': baudrate
        }],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    )
    
    # RViz 노드 (항상 실행)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        mode_arg,
        port_arg,
        baudrate_arg,
        robot_state_publisher_node,
        sine_motion_node,
        uart_controller_node,
        rviz_node
    ])
