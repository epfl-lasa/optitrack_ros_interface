from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrpn_mocap',
            executable='client_node',  # updated executable name
            name='vrpn_mocap_client',
            output='screen',
            parameters=[
                {'server': '128.178.145.105'},
                {'port': 3883},
            ],
        ),
        Node(
            package='optitrack_ros2_interface',
            executable='optitrack_transform_publisher_ros2',
            name='optitrack_ros2_interface',
            output='screen',
            parameters=[
                {'list_object': ['ball_19']},
                {'name_base': 'franka_base17'},
                {'name_base_optitrack': 'base'},
            ],
        ),
    ])
