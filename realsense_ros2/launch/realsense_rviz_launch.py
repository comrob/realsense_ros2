import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

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
            ]
        ),
        Node(
            package='realsense_ros2',
            executable='rs_d435_node',
            name='rs_d435',
            output='screen',
            parameters=[
                {'publish_depth': True},
                {'publish_pointcloud': True},
                {'is_color': True},
                {'publish_image_raw_': True},
                {'fps': 30}
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 't265_frame', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.025', '0.03', '0.0', '0.0', '0.0', 'base_link', 'camera_link_d435']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.025', '0.03', '-1.5708', '0.0', '-1.5708', 'base_link', 'camera_link_d435_pcl']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
