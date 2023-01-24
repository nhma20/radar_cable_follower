# radar_cable_follower
ROS2 package for the radar_cable_follower

For hardware setup, see:
https://github.com/nhma20/radar_cable_follower_HW

![uzh_rcf_data_example](https://user-images.githubusercontent.com/76950970/208649778-fa7fce0c-6278-4dfc-a520-ef954ad83a8b.jpg)


```sh
launch radar_cable_follower SIM_radar_power_follower.launch.py
```

## ROS2 Parameters

### Point Cloud Processing Parameters

Maximum number of concatenated pointclouds (if not using downsampling):
`ros2 param set /radar_pcl_filter_node concat_size 200`

Leaf size of voxel grid (if using downsampling):
`ros2 param set /radar_pcl_filter_node leaf_size 0.75`

Rate at which pointcloud is reduced (fixed or downsampling). 10/downsample_rate Hz, higer value is lower frequency, 1 is fastest.
`ros2 param set /radar_pcl_filter_node downsample_rate 5`

Height above ground below which all points are ignored (meters) (`float`):
`ros2 param set /radar_pcl_filter_node ground_threshold 1.5`

Points closer to the drone than this threshold are ignored (meters) (`float`):
`ros2 param set /radar_pcl_filter_node drone_threshold 0.5`

"Radius" of square around drone in which points are considered (meters) (`float`):
`ros2 param set /radar_pcl_filter_node cluster_crop_radius 20.0`

Rate at which pointcloud is cropped. 10/crop_rate Hz, higer value is lower frequency, 1 is fastest.
`ros2 param set /radar_pcl_filter_node crop_rate 5`

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

Whether sensor points upwards or downwards (`string`: `upwards`/`downwards`):
`ros2 param set /radar_pcl_filter_node sensor_upwards_or_downwards "downwards"`

Rate at which pointcloud is processed (concat, reduce, analyze). Higer value is lower frequency, 1 is fastest. (`int`):
`ros2 param set /radar_pcl_filter_node _add_crop_downsample_rate 10`

Ratio with which current powerlines are updated with new measurement (`float`):
`ros2 param set /radar_pcl_filter_node tracking_update_ratio 0.01`

Distance below which a match is considered during tracking (`float`):
`ros2 param set /radar_pcl_filter_node tracking_update_euclid_dist 1.5`




### Flight Parameters

"Proportional gain" in target yaw fractional controller (`float`):
`ros2 param set /offboard_control yaw_frac 0.25`

"Proportional gain" in target position fractional controller (`float`):
`ros2 param set /offboard_control pos_frac 0.5`

Distance to powerline during following (meters) (`float`):
`ros2 param set /offboard_control powerline_following_distance 10.0`

Speed of drone during powerline following. Negative is reverse direction (m/s) (`float`):
`ros2 param set /offboard_control powerline_following_speed 1.0`

ID of detected powerline to follow (`int`):
`ros2 param set /offboard_control powerline_following_ID -1`

Take off to height if set before entering offboard mode. If below 1, will instead align with powerlines in offboard mode (`float`):
`ros2 param set /offboard_control take_off_to_height 0.0`

