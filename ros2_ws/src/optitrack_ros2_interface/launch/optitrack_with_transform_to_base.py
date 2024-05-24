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
                {'server': '128.178.145.104'},
                {'port': 3883},
            ],
        ),
        Node(
            package='optitrack_ros2_interface',
            executable='optitrack_transform_publisher_ros2',
            name='optitrack_ros2_interface',
            output='screen',
            parameters=[
                {'list_object': ['eefUr5']},
                {'name_base': 'BaseRidgeback'},
                {'name_base_optitrack': 'base'},
            ],
        ),
    ])