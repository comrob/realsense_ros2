from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_ros2',
            executable='rs_t265_node',
            name='rs_t265',
            output='screen',
            parameters=[
                {'enable_pose_jumping': False},
                {'enable_relocalization': False},
                {'publish_fisheye': True}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
