# uosm_robot_viewer

```
sudo apt install ros-$(echo $ROS_DISTRO)-robot-state-publisher \
                 ros-$(echo $ROS_DISTRO)-joint-state-publisher \
                 ros-$(echo $ROS_DISTRO)-xacro
```

```
colcon build --packages-select uosm_robot_viewer
source install/setup.bash
ros2 launch uosm_robot_viewer default_viewer.launch.py
```