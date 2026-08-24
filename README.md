# Object Pose ROS Workspace
This repository contains a ROS workspace (ros_ws) aimed at providing the pose of an object relative to a base. Currently, it operates within the ROS (Robot Operating System) environment. Furthemore, a version compatible with ROS2 is available (ros2_ws).

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

<br>

---------------------------------------
# Multi_bot_bench Application
This branch is created initially to record object poses in csv using optitrack and use it for multi_bot_bench project.

It adds two functionalities, record_trajectory.py and record_multiple_trajectory.py. The multi_bot_bench project used ROS1 version of optitrack (ros_ws).

## IP Address
The left side optitrack of the robot room (where Kuka is) has ip address: 128.178.145.104, and right side of optitrack has ip address: 128.178.145.172. 

Make sure to change the ending of ip address accordingly on line 3 of 'optitrack_with_transform_to_base.launch' launch file, or overwrite it using the command (port number should be 3883 for both):

```bash
rosrun vrpn_client_ros vrpn_client_node  _server:=128.178.145.172  _port:=3883  
```


## Terminal Commands 
### Terminal 1
clone git@github.com:epfl-lasa/optitrack_ros_interface.git 

```bash
cd /path/to/optitrack_ros_interface/ros_ws/docker 
bash start_optitrack.sh 
```

You will notice a lot of logging messages appearing, and all active objects being created. Do not press anything but enter in the same docker container from a new terminal.


### Terminal 2
```bash
docker exec -it epfl-lasa-optitrack_ros1 bash
```

When terminal 2 now shows a bunch of logs, terminal 1 will clear up. 

### Back to Terminal 1
Once terminal 1 has cleared up after executing the container from terminal 2, you can use commands to give commands.

- To check the current active objects, check rostopic:

```bash
rostopic list
```

- Echo current position & orientation (quaternion):
```bash
rostopic echo /vrpn_client_node/object_name/pose
```

- record one object trajectory (change the parameters accordingly):
```bash
rosrun optitrack_ros_interface record_trajectory.py   _topic:=/vrpn_client_node/object_name/pose   _output_directory:=/home/ros/ros_ws/data   _file_name:=filename.csv
```

- record multiple object trajectories (change the parameters accordingly):
```bash
rosrun optitrack_ros_interface record_multiple_trajectory.py   _topics:="[/vrpn_client_node/object_1/pose, /vrpn_client_node/object_2/pose]"   _output_directory:=/home/ros/ros_ws/data    _file_name:=object1_and_object2.csv
```

