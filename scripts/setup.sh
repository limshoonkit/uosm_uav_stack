#!/bin/bash
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

sudo apt update && sudo apt upgrade -y

sudo apt install libpcl-dev libeigen3-dev libgtsam-dev -y
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