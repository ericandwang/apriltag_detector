from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the zed_camera.launch.py file in the zed_wrapper package
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'name': 'zed2i'
        }.items()
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_pub_head2_camera',
        arguments=[
            '-0.01', '-0.060', '0.015',  # x y z
            '0', '0', '0',               # roll pitch yaw (radians)
            'link_head_2', 'camera_right'
        ],
        output='screen'
    )

    apriltag_node = Node(
        package='apriltag_detector',
        executable='apriltag_detector_node',
        name='apriltag_detector_node',
        output='screen'
    )

    return LaunchDescription([
        zed_launch,
        static_tf,
        apriltag_node
    ])
