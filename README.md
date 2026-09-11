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

```bash
$EDITOR .env                              # Motive IP + rigid bodies
docker compose --profile ros2 up --build
```

There is no default profile, so a bare `docker compose up` starts nothing — pick
`--profile ros2` or `--profile ros1`. Running both at once means two VRPN clients
polling the same rigid bodies.

```bash
docker compose --profile ros2 up -d            # detached
docker compose --profile ros2 logs -f          # follow
docker compose --profile ros2 restart ros2     # after editing config/
docker compose --profile ros2 exec ros2 bash   # a shell
docker compose --profile ros2 down             # stop
```