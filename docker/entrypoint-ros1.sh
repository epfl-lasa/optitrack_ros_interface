#!/usr/bin/env bash
# See entrypoint-ros2.sh for why this is `set -eo pipefail` without `-u`.
set -eo pipefail

source /opt/ros/noetic/setup.bash
if [ -f "${OVERLAY_WS}/devel/setup.bash" ]; then
  source "${OVERLAY_WS}/devel/setup.bash"
fi

# Advertise this container's bridge IP for XMLRPC and TCPROS callbacks.
# Preserve an explicit override; never pass an empty ROS_IP to roscpp.
if [ -z "${ROS_IP:-}" ]; then
  read -r -a container_ips <<< "$(hostname -I)"
  export ROS_IP="${container_ips[0]:-}"
  if [ -z "$ROS_IP" ]; then
    echo "[optitrack] Could not determine container IP; set ROS_IP explicitly." >&2
    exit 1
  fi
fi
unset ROS_HOSTNAME
echo "[optitrack] ROS_IP=${ROS_IP}"

echo "[optitrack] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

exec "$@"
