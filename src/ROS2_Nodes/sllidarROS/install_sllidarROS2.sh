git clone git@github.com:Slamtec/sllidar_ros2.git

source /opt/ros/jazzy/setup.bash
sudo apt install -y colcon
sudo apt install -y  ros-jazzy-rmw-fastrtps-cpp
colcon build --symlink-install

echo "source ~/Xelerator-Soccer-OpenSource/src/ROS2_Nodes/sllidarROS/install/setup.bash" >> ~/.bashrc

source ~/.bashrc