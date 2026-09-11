"""Launch the VRPN client and the base-frame transform publisher.

Tracked defaults live in config/optitrack.ros2.yaml. Site-specific values come
from the environment (.env via docker compose) and override the config file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _csv(name, default=''):
    """Split a comma-separated environment value into a list, dropping empties.

    Done in plain Python rather than with EnvironmentVariable substitutions:
    `launch` has no split substitution, so the substitution route would need a
    custom Substitution class or a PythonExpression eval, and its failure mode is
    a silent single-element list like ['a,b,c'].
    """
    return [x.strip() for x in os.environ.get(name, default).split(',') if x.strip()]


def generate_launch_description():
    pkg_share = get_package_share_directory('optitrack_ros2_interface')
    default_config = os.environ.get(
        'OPTITRACK_CONFIG',
        os.path.join(pkg_share, 'config', 'optitrack.ros2.yaml'))

    objects = _csv('OPTITRACK_OBJECTS')
    base_object = os.environ.get('OPTITRACK_BASE_OBJECT', '')

    if not objects:
        raise RuntimeError('OPTITRACK_OBJECTS is empty; set it in .env')
    if not base_object:
        raise RuntimeError('OPTITRACK_BASE_OBJECT is empty; set it in .env')

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        LogInfo(msg='optitrack: base={} objects={}'.format(base_object, objects)),

        Node(
            package='vrpn_mocap',
            executable='client_node',
            name='vrpn_mocap_client',
            output='screen',
            parameters=[
                config_file,
                {
                    'server': os.environ.get('OPTITRACK_SERVER', '127.0.0.1'),
                    'port': int(os.environ.get('OPTITRACK_PORT', '3883')),
                },
            ],
        ),
        Node(
            package='optitrack_ros2_interface',
            executable='optitrack_transform_publisher_ros2',
            name='optitrack_ros2_interface',
            output='screen',
            parameters=[
                config_file,
                {
                    'list_object': objects,
                    'name_base': base_object,
                    # base_frame is left unset: the node defaults it to
                    # name_base, which is the frame_id downstream expects.
                    # Override it in config/optitrack.ros2.yaml if ever needed.
                },
            ],
        ),
    ])
