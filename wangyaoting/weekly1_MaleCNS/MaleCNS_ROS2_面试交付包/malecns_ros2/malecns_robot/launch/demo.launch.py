from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('malecns_robot'))
    world = str(share / 'worlds' / 'arena.world')
    robot = str(share / 'models' / 'robot.sdf')
    rviz = str(share / 'rviz' / 'demo.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.5'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('log_path', default_value='/tmp/malecns_run.csv'),
        ExecuteProcess(cmd=['gzserver', '--verbose', '-u', world,
                            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                       condition=IfCondition(LaunchConfiguration('paused')), output='screen'),
        ExecuteProcess(cmd=['gzserver', '--verbose', world,
                            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                       condition=UnlessCondition(LaunchConfiguration('paused')), output='screen'),
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui')),
                       output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'malecns_bot', '-file', robot,
                        '-x', '-2.5', '-y', '0', '-z', '0.18']),
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['0', '0', '0.12', '0', '0', '0', 'base_link', 'laser_link']),
        Node(package='malecns_robot', executable='controller', output='screen',
             parameters=[{'use_sim_time': True,
                          'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
                          'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
                          'log_path': LaunchConfiguration('log_path')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
