# RGB-D SLAM with RTABMAP

https://youtu.be/3A4Ysf9erTo

## How to run SLAM using Rosbag:
0. Set up
```
roscore
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 zed2_camera_center odom
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 zed2_camera_center zed2_imu_link
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 VLP16-base_link camera_link
```
1. Mapping mode
```
roslaunch rtabmap_ros rtabmap.launch \
rtabmap_args:="--delete_db_on_start" \
rgb_topic:=/zed2/zed_node/rgb/image_rect_color \
depth_topic:=/zed2/zed_node/depth/depth_registered \
camera_info_topic:=/zed2/zed_node/depth/camera_info \
odom_topic:=/zed2/zed_node/odom \
imu_topic:=/zed2/zed_node/imu/data \
visual_odometry:=false \
frame_id:=zed2_camera_center \
approx_sync:=true \
rgbd_sync:=true \
approx_rgbd_sync:=false \
queue_size:=70
rosparam set use_sim_time true
rosbag play [bag file] --clock
```
2. Localization mode
```
roslaunch rtabmap_ros rtabmap.launch \
rgb_topic:=/zed2/zed_node/rgb/image_rect_color \
depth_topic:=/zed2/zed_node/depth/depth_registered \
camera_info_topic:=/zed2/zed_node/depth/camera_info \
odom_topic:=/zed2/zed_node/odom \
imu_topic:=/zed2/zed_node/imu/data \
visual_odometry:=false \
frame_id:=zed2_camera_center \
approx_sync:=true \
rgbd_sync:=true \
approx_rgbd_sync:=false \
queue_size:=70 \
localization:=true
rosparam set use_sim_time true
rosbag play [bag file] --clock
```