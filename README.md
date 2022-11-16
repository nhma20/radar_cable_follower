# radar_cable_follower
ROS2 package for the radar_cable_follower

```sh
launch radar_cable_follower SIM_radar_power_follower.launch.py
```

## ROS2 Parameters

### Point Cloud Processing Parameters

Maximum number of concatenated pointclouds (if not using downsampling):
`ros2 param set /radar_pcl_filter_node concat_size 200`

Leaf size of voxel grid (if using downsampling):
`ros2 param set /radar_pcl_filter_node leaf_size 0.75`

Height above ground below which all points are ignored (meters) (`float`):
`ros2 param set /radar_pcl_filter_node ground_threshold 1.5`

Points closer to the drone than this threshold are ignored (meters) (`float`):
`ros2 param set /radar_pcl_filter_node drone_threshold 0.5`

"Radius" of square around drone in which points are considered (meters) (´int´):
`ros2 param set /radar_pcl_filter_node cluster_crop_radius 20`

Parallelism constraint when doing 3D line fit (degrees) (`float`):
`ros2 param set /radar_pcl_filter_node line_model_parallel_angle_threshold 10.0`

Maximum distance when looking for inliers during 3D line fit (meters) (`float`):
`ros2 param set /radar_pcl_filter_node line_model_distance_threshold 1.5`

Minimum number of inliers required for a line model (`float`):
`ros2 param set /radar_pcl_filter_node line_model_inlier_threshold 10.0`

Whether to use voxel grid downsampling or remove oldest points after `concat_size` is reached (`string`):
`ros2 param set /radar_pcl_filter_node voxel_or_time_concat "voxel"`

Whether to use highest point or line fit model following (`string`):
`ros2 param set /radar_pcl_filter_node line_or_point_follow "point"`

Whether to use statistical outlier filtering during point following (`int`):
`ros2 param set /radar_pcl_filter_node point_follow_outlier_filter 1`


### Flight Parameters

"Proportional gain" in target yaw fractional controller (`float`):
`ros2 param set /offboard_control yaw_frac 0.25`

"Proportional gain" in target position fractional controller (`float`):
`ros2 param set /offboard_control pos_frac 0.5`

Distance to powerline during following (meters) (`float`):
`ros2 param set /offboard_control powerline_following_distance 10.0`

Speed of drone during powerline following. Negative is reverse direction (m/s) (`float`):
`ros2 param set /offboard_control powerline_following_speed 1.0`

