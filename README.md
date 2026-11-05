# optitrack_ros_interface

Republishes OptiTrack/Motive rigid-body poses — received over VRPN — in a robot
base frame instead of the mocap world frame.

For each rigid body `X`, it publishes `T_base_X = T_world_base⁻¹ · T_world_X` on
`<X>/pose_from_<base>`.

| Workspace  | ROS distro   | VRPN client       | Profile |
|------------|--------------|-------------------|---------|
| `ros2_ws/` | Humble       | `vrpn_mocap`      | `ros2`  |
| `ros_ws/`  | Noetic (EOL) | `vrpn_client_ros` | `ros1`  |

## Configuration

All site-specific settings - the VRPN server address, the list of rigid
bodies to republish, and which one is the robot base - live in
[`config/optitrack_cfg.yaml`](config/optitrack_cfg.yaml).

## Quick start

There is no default profile, so a bare `docker compose up` starts nothing. Pick
`--profile ros2` or `--profile ros1`.

ROS1:
```bash
docker compose --profile ros1 up --build
```

The container uses host networking. Two ROS master setups are possible:

- **Local master (default):** `ROS_MASTER_URI` is unset, so `roslaunch`
  starts its own master on `localhost:11311`. `ROS_IP` defaults to the host's
  first IP address.
- **External master** (e.g. a robot container): uncomment the `environment`
  block of the `ros1` service in `docker-compose.yml` and set `ROS_MASTER_URI`
  to the master and `ROS_IP` to an address of this host the master can reach.

Only one instance of these nodes may run per master: identical node names make
them shut each other down.

ROS2:
```bash
docker compose --profile ros2 up --build
```