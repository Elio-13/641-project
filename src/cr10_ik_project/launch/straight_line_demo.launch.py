import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
            output='screen'
        ),
    ])
