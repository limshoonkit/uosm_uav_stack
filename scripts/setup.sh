#!/bin/bash
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

sudo apt update && sudo apt upgrade -y

# Kitware APT repo for CMake >= 3.24 (required by kiss-icp/Sophus)
# Adds the repo and upgrades cmake in-place; never removes cmake (would cascade-remove ROS)
if ! dpkg -l kitware-archive-keyring &>/dev/null; then
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
        | gpg --dearmor - \
        | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' \
        | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
    sudo apt update
    sudo apt install -y --only-upgrade cmake cmake-data
    sudo apt install -y kitware-archive-keyring
fi

sudo apt install libpcl-dev libeigen3-dev -y
sudo pip3 install -U jetson-stats
sudo apt install \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-pcl-ros \
    ros-humble-camera-info-manager \
    ros-humble-joint-state-publisher \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ament-cmake \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-foxglove-bridge \
    ros-humble-foxglove-msgs \
    -y
sudo "$(dirname "$0")/install_geolib.sh"
sudo rosdep init
rosdep update
rosdep install --from-paths "$REPO_ROOT/src/third_party/acados_vendor_ros2" -y --ignore-src
rosdep install --from-paths "$REPO_ROOT/src/perception_module/zed-ros2-wrapper" -y --ignore-src
