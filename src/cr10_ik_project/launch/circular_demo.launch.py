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
        DeclareLaunchArgument('center_x', default_value='-0.55'),
        DeclareLaunchArgument('center_y', default_value='0.0'),
        DeclareLaunchArgument('center_z', default_value='0.9'),
        DeclareLaunchArgument('radius', default_value='0.4'),
        DeclareLaunchArgument('steps', default_value='300'),

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
            executable='circular_ik',
            parameters=[{
                'center_x': LaunchConfiguration('center_x'),
                'center_y': LaunchConfiguration('center_y'),
                'center_z': LaunchConfiguration('center_z'),
                'radius': LaunchConfiguration('radius'),
                'steps': LaunchConfiguration('steps'),
            }],
            output='screen'
        ),
    ])
