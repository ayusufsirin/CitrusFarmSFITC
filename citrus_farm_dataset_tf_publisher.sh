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
python3 odom_to_tf.py&
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link zed2i_base_link&
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link&
rosrun tf2_ros static_transform_publisher 0.04 0 0.3787 0 0 0 base_link velodyne&
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom /jackal_velocity_controller/odom&
rosrun tf2_ros static_transform_publisher 0 0 0 0 1.57079633 0 base_link microstrain_link&
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 zed2i_base_link zed2i_left_camera_optical_frame&
rosrun tf2_ros static_transform_publisher 0 0 0 0 1.57079633 0 zed2i_base_link zed2i_imu_link&

# Wait for background processes to finish
wait
