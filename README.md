# GruntRobot
Autonomous  robot

***To create ws***
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source catkin_ws/devel/setup.bash" >> ~/.bashrc
echo $ROS_PACKAGE_PATH
********************

sudo apt install python-is-python3 //after installing ros

***To run Audrino***
sudo chmod 666 /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0

sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-laser-scan-matcher
sudo apt install ros-noetic-map-server
