# syntax=docker/dockerfile:1.7
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG USERNAME=ros
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      libeigen3-dev \
      # /etc/protocols; without it getprotobyname() fails and VRPN cannot
      # connect at all. The ros base images do not ship it.
      netbase \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-vrpn-mocap \
    && rm -rf /var/lib/apt/lists/*

# The uid need not match the host's: the only mount is ./config, read-only.
RUN useradd --create-home --shell /bin/bash ${USERNAME}

# Container logs are a pipe, not a tty, so logging would otherwise be buffered.
ENV RCUTILS_LOGGING_BUFFERED_STREAM=0
ENV OVERLAY_WS=/home/${USERNAME}/ros2_ws
ENV OPTITRACK_CONFIG=/opt/optitrack/config/optitrack.ros2.yaml

COPY --chmod=0755 docker/entrypoint-ros2.sh /opt/optitrack/entrypoint-ros2.sh
COPY config/ /opt/optitrack/config/
COPY --chown=${USERNAME}:${USERNAME} ros2_ws/src/ ${OVERLAY_WS}/src/
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}
WORKDIR ${OVERLAY_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Sources the overlay only. Launching is the entrypoint's job, so `exec bash`
# gives a shell rather than a second copy of the stack.
RUN printf '%s\n' \
      'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash' \
      'if [ -f "$OVERLAY_WS/install/setup.bash" ]; then source "$OVERLAY_WS/install/setup.bash"; fi' \
      >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/opt/optitrack/entrypoint-ros2.sh"]
CMD ["ros2", "launch", "optitrack_ros2_interface", "optitrack.launch.py"]
