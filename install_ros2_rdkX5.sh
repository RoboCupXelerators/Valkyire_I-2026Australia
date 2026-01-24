sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Finished adding ROS2 repository."

sudo apt update
sudo apt upgrade # 可选，但建议更新已安装的包
sudo apt install ros-humble-desktop

echo "Finished install ROS2 repository.✅"

sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions python3-rosdep

echo "initializing rosdep...✅"

sudo rosdep init
rosdep update

source /opt/ros/humble/setup.bash

echo "initializing ros...✅"

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
