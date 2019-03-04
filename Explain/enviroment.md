# 环境配置

#### ROS2(ubantu18+ros2_crystal)
````
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

export CHOOSE_ROS_DISTRO=crystal
sudo apt update

sudo apt install ros-$CHOOSE_ROS_DISTRO-desktop

sudo apt install python3-argcomplete
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional RMW implementations¶
sudo apt update
sudo apt install ros-$ROS_DISTRO-rmw-opensplice-cpp # for OpenSplice
sudo apt install ros-$ROS_DISTRO-rmw-connext-cpp # for RTI Connext (requires license agreement)


#如果要和ROS1建立连接
#Install additional packages using ROS 1 packages
sudo apt update
sudo apt install ros-$ROS_DISTRO-ros1-bridge
````

#### 编译工具colcon  
[colcon](colcon.md)  

#### yaml-cpp  
> ```` sudo apt-get install libyaml-cpp-dev ````    