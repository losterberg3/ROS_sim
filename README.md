# Setup

Starting a headless (without the GUI) ROS2 simulation with gazebo.

Install docker for desktop. Then create a container with the ROS2 software and a mounted folder "ROS_sim". 
You're now in an interactive shell.

    docker run -it --name ROS_sim -v ~/ROS_sim:/root/ROS_sim ros:humble

This container does not remove when you stop. To have it be removed after each use, use tag -rm.

Then install turtlebot3, or any bot:

    apt update && apt install -y ros-humble-turtlebot3*

Then, to have the ROS2 files sourced and the right turtlebot at startup, do:

    nano ~/.bashrc

Add these lines:

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger

Or whatever bot model you like. Then, to launch a world without the gui:

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false

Then you're good to go. If you want to enter a new shell on an already running container:

    docker exec -it <Container ID, i.e. 9581fd25802e> bash

To restart the container after it's been stopped (for container named ROS_sim):

    docker start -ai ROS_sim

