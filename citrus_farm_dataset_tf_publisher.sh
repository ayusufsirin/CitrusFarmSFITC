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

rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100&
python3 odom_to_tf.py&
rosrun tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link zed2i_left_camera_optical_frame&
rosrun tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link velodyne&
rosrun tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link zed2i_base_link&
rosrun tf2_ros static_transform_publisher 0 0 0.1 0 0 0 zed2i_base_link zed2i_imu_link&

# Wait for background processes to finish
wait
