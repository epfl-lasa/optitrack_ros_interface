# optitrack_ros_interface

Republishes OptiTrack/Motive rigid-body poses — received over VRPN — in a robot
base frame instead of the mocap world frame.

For each rigid body `X`, it publishes `T_base_X = T_world_base⁻¹ · T_world_X` on
`<X>/pose_from_<base>`.

| Workspace  | ROS distro   | VRPN client       | Profile |
|------------|--------------|-------------------|---------|
| `ros2_ws/` | Humble       | `vrpn_mocap`      | `ros2`  |
| `ros_ws/`  | Noetic (EOL) | `vrpn_client_ros` | `ros1`  |

## Quick start

There is no default profile, so a bare `docker compose up` starts nothing. Pick
`--profile ros2` or `--profile ros1`.

ROS1:
```bash
docker compose --profile ros1 up --build
```

ROS2:
```bash
docker compose --profile ros2 up --build
```