```
ros2 launch jetson_gscam2 csi_cam.launch.py
```

```
ros2 bag play /home/nvidia/uav_autonomy_stack/bags/csi_cam_recording/csi_cam_recording_0.db3 --qos-profile-overrides-path /home/nvidia/uav_autonomy_stack/src/perception_module/jetson_gscam2/config/qos_replay_overrides.yaml


source install/setup.bash
ros2 run jetson_gscam2 decoder_node.py
```
