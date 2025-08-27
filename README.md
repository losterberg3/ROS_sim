# Setup

Install docker for desktop. Then create a container with the ROS2 software and a mounted folder "ROS_sim".

    docker run -it --name ROS_sim -v ~/ROS_sim:/root/ROS_sim ros:humble

This container does not remove when you stop. To have it be removed after each use, use tag {-rm}.

docker start -ai ROS_sim

If you deleted the container, how I entered the container the very first time:



And install turtlebot3:

apt update && apt install -y ros-humble-turtlebot3*


Then, ROS2 and turtlebot will startup automatically because of settings.

To edit these settings,

nano ~/.bashrc

Change these lines at the bottom:

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger


Then, to launch the world without the gui:

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false


To control the robot:

ros2 run turtlebot3_teleop teleop_keyboard


To enter an already running container:

docker exec -it 9581fd25802e bash
