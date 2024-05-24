# Handle ros distro
ARG ROS_DISTRO=humble

FROM ghcr.io/aica-technology/ros2-ws:${ROS_DISTRO}

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
USER root
RUN if [ "HOST_GID" != "1000"] ; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# Set environment variables
ENV HOME /home/${USER}
ENV ROS_WS ${HOME}/ros2_ws

# Set the working directory
WORKDIR ${ROS_WS}

# Setup git identity
RUN git config --global user.name "${GIT_NAME}"
RUN git config --global user.email "${GIT_EMAIL}"

# Setup python version for noetic
RUN sudo apt update
RUN sudo apt install python-is-python3

### Add a few tools
RUN sudo apt-get update && sudo apt-get install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    netbase \
    ros-humble-vrpn-mocap \
    && sudo apt-get upgrade -y && sudo apt-get clean

RUN sudo pip install scipy
### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean 


WORKDIR /home/${ROS_WS} && mkdir -p src/optitrack_ros2_interface

# Clone the repository and copy the necessary files
RUN git clone https://github.com/epfl-lasa/optitrack_ros_interface.git /tmp/optitrack_ros_interface \
    && cp -r /tmp/optitrack_ros_interface/ros2_ws/src/optitrack_ros2_interface ${ROS_WS}/src/ \
    && rm -rf /tmp/optitrack_ros_interface

### Add environement variables to bashrc
WORKDIR /home/${USER}

# Give bashrc back to user
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

### Build ros2 workspace
WORKDIR /${ROS_WS}/
RUN source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash;


RUN echo "source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    ros2 launch optitrack_ros2_interface optitrack_with_transform_to_base.py" >> ${HOME}/.bashrc ;



