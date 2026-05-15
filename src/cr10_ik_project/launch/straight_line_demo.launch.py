import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('dobot_rviz'),
        'urdf',
        'cr10_robot.urdf',
    )
    rviz_config = os.path.join(
        get_package_share_directory('cr10_ik_project'),
        'rviz',
        'cr10_demo.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='-0.65'),
        DeclareLaunchArgument('start_y', default_value='0.35'),
        DeclareLaunchArgument('start_z', default_value='1.0'),
        DeclareLaunchArgument('end_x', default_value='-0.65'),
        DeclareLaunchArgument('end_y', default_value='-0.35'),
        DeclareLaunchArgument('end_z', default_value='1.0'),
        DeclareLaunchArgument('steps', default_value='200'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        Node(
            package='cr10_ik_project',
            executable='straight_line_ik',
            parameters=[{
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'start_z': LaunchConfiguration('start_z'),
                'end_x': LaunchConfiguration('end_x'),
                'end_y': LaunchConfiguration('end_y'),
                'end_z': LaunchConfiguration('end_z'),
                'steps': LaunchConfiguration('steps'),
            }],
            output='screen'
        ),
    ])
