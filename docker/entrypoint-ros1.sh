#!/usr/bin/env bash
# See entrypoint-ros2.sh for why this is `set -eo pipefail` without `-u`.
set -eo pipefail

source /opt/ros/noetic/setup.bash
if [ -f "${OVERLAY_WS}/devel/setup.bash" ]; then
  source "${OVERLAY_WS}/devel/setup.bash"
fi

# roscpp rejects an empty ROS_IP outright, so drop it rather than pass it on.
# ROS_IP and ROS_HOSTNAME are mutually exclusive.
if [ -z "${ROS_IP:-}" ]; then
  unset ROS_IP
else
  unset ROS_HOSTNAME
  echo "[optitrack] ROS_IP=${ROS_IP}"
fi

echo "[optitrack] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

exec "$@"
