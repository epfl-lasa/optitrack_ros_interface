# syntax=docker/dockerfile:1.7
ARG ROS_DISTRO=noetic

# Noetic is EOL. This tag was removed from docker-library/official-images: never
# rebuilt, no security updates, apt source frozen at snapshots.ros.org. Pinned by
# digest so a retag cannot change the build.
#   refresh: docker buildx imagetools inspect ros:noetic-ros-base
FROM ros:noetic-ros-base@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

ARG ROS_DISTRO
ARG USERNAME=ros
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# No rosdep: Noetic is flagged end-of-life in rosdistro, so `rosdep install`
# fails on unresolvable keys. Explicit packages cannot rot that way.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libeigen3-dev \
      # /etc/protocols; without it getprotobyname() fails and VRPN cannot
      # connect at all. The ros base images do not ship it.
      netbase \
      python-is-python3 \
      ros-noetic-vrpn-client-ros \
    && rm -rf /var/lib/apt/lists/*

# The uid need not match the host's: the only mount is ./config, read-only.
RUN useradd --create-home --shell /bin/bash ${USERNAME}

# Container logs are a pipe, not a tty, so logging would otherwise be buffered.
ENV ROSCONSOLE_STDOUT_LINE_BUFFERED=1
ENV OVERLAY_WS=/home/${USERNAME}/ros_ws
ENV OPTITRACK_CONFIG=/opt/optitrack/config/optitrack_cfg.yaml

COPY --chmod=0755 docker/entrypoint-ros1.sh /opt/optitrack/entrypoint-ros1.sh
COPY config/ /opt/optitrack/config/
COPY --chown=${USERNAME}:${USERNAME} ros_ws/src/ ${OVERLAY_WS}/src/
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}
WORKDIR ${OVERLAY_WS}
RUN source /opt/ros/noetic/setup.bash \
 && catkin_make -DCMAKE_BUILD_TYPE=Release

# Sources the overlay only; launching is the entrypoint's job.
RUN printf '%s\n' \
      'source /opt/ros/noetic/setup.bash' \
      'if [ -f "$OVERLAY_WS/devel/setup.bash" ]; then source "$OVERLAY_WS/devel/setup.bash"; fi' \
      >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/opt/optitrack/entrypoint-ros1.sh"]
CMD ["roslaunch", "optitrack_ros_interface", "optitrack.launch"]
