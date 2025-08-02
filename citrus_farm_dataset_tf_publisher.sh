#!/bin/bash

source /opt/ros/noetic/setup.bash

# Function to clean up background processes
cleanup() {
    echo "Killing all background processes..."
    kill $(jobs -p)
    exit 0
}

# Trap SIGINT (Ctrl+C) and call cleanup
trap cleanup SIGINT

# https://ucr-robotics.github.io/Citrus-Farm-Dataset/calibration.html
# TODO: Set transform offsets
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100&
python3 $(dirname $0)/odom_to_tf.py&
rosrun tf2_ros static_transform_publisher 0.0400  0.0000  0.3787  0.0000 0.0000  0.0000  1.0000  base_link        velodyne&
rosrun tf2_ros static_transform_publisher -0.2200 0.0000  0.1530  0.0000 0.0000  0.0000  1.0000  velodyne         gps_rtk&
rosrun tf2_ros static_transform_publisher 0.2178  0.0049  -0.0645 0.5076 -0.4989 0.4960  -0.4974 velodyne         flir_blackfly&
rosrun tf2_ros static_transform_publisher -0.0061 0.0157  -0.1895 0.4987 0.5050  -0.4987 0.4977  flir_blackfly    microstrain_link&
rosrun tf2_ros static_transform_publisher -0.0663 0.0956  -0.0161 0.0020 -0.0081 0.0031  1.0000  flir_blackfly    zed2i_base_link&
rosrun tf2_ros static_transform_publisher 0.0000 0.0000  0.0000 0.0000 0.0000 0.0000  1.0000     zed2i_base_link  zed2i_rgb_left&

rosrun tf2_ros static_transform_publisher 0.0000 0.0000  0.0000 0.0000 0.0000 0.0000  1.0000     zed2i_base_link  zed2i_left_camera_optical_frame&
#rosrun tf2_ros static_transform_publisher 0.0000 0.0000  0.0000 1.5708 -1.5708 0.0000     zed2i_base_link  zed2i_left_camera_optical_frame&
#rosrun tf2_ros static_transform_publisher 0.0000 0.0000  0.0000 0.0000 -1.5708 0.0000     zed2i_base_link  zed2i_base_link_pitch&
#rosrun tf2_ros static_transform_publisher 0.0000 0.0000  0.0000 -1.5708 0.0000 0.0000     zed2i_base_link_pitch  zed2i_base_link_roll&
#rosrun tf2_ros static_transform_publisher 0.0000 0.0000  0.0000 0.0000 0.0000 3.1416     zed2i_base_link_roll  zed2i_left_camera_optical_frame&

rosrun tf2_ros static_transform_publisher 0.0000 0.0000  0.0000 0.0000 0.0000 0.0000  1.0000     zed2i_base_link  zed2i_imu_link&
rosrun tf2_ros static_transform_publisher 0.1198  -0.0003 -0.0046 0.0013 0.0013  0.0000  1.0000  zed2i_rgb_left   zed2i_rgb_right&
rosrun tf2_ros static_transform_publisher 0.0251  -0.0948 -0.0203 0.0026 -0.0032 0.0059  1.0000  zed2i_rgb_right  flir_adk&
rosrun tf2_ros static_transform_publisher -0.1608 -0.0046 -0.0138 0.0028 0.0186  -0.0094 0.9998  flir_adk         mapir&

# Wait for background processes to finish
wait
