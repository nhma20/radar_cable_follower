# radar_cable_follower
ROS2 package for the radar_cable_follower

```sh
launch radar_cable_follower SIM_radar_power_follower.launch.py
```

### ROS2 Parameters
Maximum number of concatenated pointclouds (if not using downsampling):
```sh
ros2 param set /radar_pcl_filter_node concat_size 200
```

Leaf size of voxel grid (if using downsampling):
```sh
ros2 param set /radar_pcl_filter_node leaf_size 0.75
```

Distance threshold to line model:
```sh
ros2 param set /radar_pcl_filter_node _model_thresh 0.5
```
