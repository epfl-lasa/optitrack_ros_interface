#!/usr/bin/env bash
# No `set -u`: ROS setup.bash dereferences unset variables and would abort.
# `if` rather than `[ -f x ] && source x`, which under `set -e` would exit 1.
set -eo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
if [ -f "${OVERLAY_WS}/install/setup.bash" ]; then
  source "${OVERLAY_WS}/install/setup.bash"
fi

echo "[optitrack] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-<default>}"

exec "$@"
