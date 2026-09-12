"""Launch the VRPN client and the base-frame transform publisher.

Tracked config, including rig values (server, objects, base) and driver
settings, lives in config/optitrack_cfg.yaml. Edit it directly for a given
site.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('optitrack_ros2_interface')
    config_path = os.environ.get(
        'OPTITRACK_CONFIG',
        os.path.join(pkg_share, 'config', 'optitrack_cfg.yaml'))

    # config/optitrack_cfg.yaml is written in ROS1's flat rosparam shape only
    # (see its header comment) and read here with plain PyYAML rather than
    # handed to the node as a --params-file: rcl_yaml_param_parser refuses to
    # parse a file with any top-level key that isn't itself a ros__parameters
    # block (verified: "Cannot have a value before ros__parameters"), and it
    # does not support YAML anchors either (verified: "Will not support
    # aliasing"), so this is the only way for both ROS versions to read the
    # same declaration of a site value.
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    vrpn_params = cfg.get('vrpn_client_node', {})
    transform_params = cfg.get('optitrack_ros_interface', {})

    # This workspace's own VRPN driver node name. Not read from the config
    # file: vrpn_namespace there is ROS1's own node name ("/vrpn_client_node"),
    # which does not apply here -- this is launch wiring, not a site setting.
    vrpn_node_name = 'vrpn_mocap_client'

    return LaunchDescription([
        Node(
            package='vrpn_mocap',
            executable='client_node',
            name=vrpn_node_name,
            output='screen',
            parameters=[{
                'server': vrpn_params.get('server', '127.0.0.1'),
                'port': vrpn_params.get('port', 3883),
                'frame_id': vrpn_params.get('frame_id', 'world'),
                # keep above Motive's rate, else samples arrive in bursts
                'update_freq': 1000.0,
                'refresh_freq': 1.0,             # rescan for new rigid bodies, Hz
                # BEST_EFFORT: echo needs --qos-reliability best_effort
                'sensor_data_qos': True,
                'multi_sensor': False,
                'use_vrpn_timestamps': False,    # stamp on receipt; true needs a clock-synced Motive PC
            }],
        ),
        Node(
            package='optitrack_ros2_interface',
            executable='optitrack_transform_publisher_ros2',
            name='optitrack_ros2_interface',
            output='screen',
            parameters=[{
                'vrpn_namespace': vrpn_node_name,
                'list_object': transform_params.get('list_object', []),
                'name_base': transform_params.get('name_base', ''),
            }],
        ),
    ])
