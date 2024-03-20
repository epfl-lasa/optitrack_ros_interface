# Object Pose ROS Workspace
This repository contains a ROS workspace (ros_ws) aimed at providing the pose of an object relative to a base. Currently, it operates within the ROS (Robot Operating System) environment. However, a version compatible with ROS2 is planned for the near future.

## Purpose
The primary objective of this workspace is to facilitate the determination of an object's pose concerning a base. This functionality is crucial in various robotics applications where precise spatial information is required for navigation, manipulation, or interaction tasks.

## Usage
To utilize the functionality provided by this repository, follow these steps:

1. **Clone the Repository:** Clone this repository to your local machine using the following command:
    ```bash
    git clone https://github.com/epfl-lasa/optitrack_ros_interface.git
    ```

2. **Navigate to the Docker Folder:** Enter the `docker` directory within the cloned repository.
    ```bash
    cd ros_ws/docker
    ```

3. **Build the Docker Image:** Execute the `build_docker.sh` script using Bash to build the Docker image:
    ```bash
    bash build_docker.sh
    ```

4. **Run the Docker Container:** After the Docker image is built successfully, run the `start_optitrack.sh` script to start the application:
    ```bash
    bash start_optitrack.sh
    ```

## Docker Configuration
The Docker folder within this repository contains all necessary files to facilitate easy deployment and usage. By following the provided instructions, users can seamlessly set up and run the application within a Docker container, ensuring consistency across different environments.You can install automatically all the docker dependency by runing the install_docker.sh file. When the Docker is started, it automatically runs the `roslaunch` that will stream the poses.

## Configuration
To configure the application, modify the following lines in the launch file:

<arg name="server" default="IP"/>
<arg name="list_object" default="['/vrpn_client_node/OBJECT1/pose','/vrpn_client_node/OBJECT2/pose',]"/>

<arg name="name_base_optitrack" default="base"/>
<arg name="name_base" default="/vrpn_client_node/BASE/pose"/>

Replace BASE with the name of your base and OBJECT1, OBJECT2, etc., with the names of your objects accordingly of the names in motive.
You neeed aswell to replace in server, the IP adress of the computer using Motive.
## Future Development

Currently, the workspace is tailored for ROS. However, a version compatible with ROS2 is in the pipeline.
## Feedback and Contributions

Feedback, suggestions, and contributions are highly appreciated. If you encounter any issues, have ideas for improvements, or wish to contribute to the development of this workspace, feel free to open an issue or submit a pull request. Your input is valuable in enhancing the functionality and usability of this repository.
## License

This repository is licensed under the Apache License 2.0 license. Refer to the LICENSE file for more information.
## Authors/Maintainers

    Tristan Bonato: tristan.bonato@epfl.ch, @bonato47 on GitHub.
    Bruno Agostinho Da Costa: bruno.agostinhodacosta@epfl.ch, @Brunoadc on GitHub.


