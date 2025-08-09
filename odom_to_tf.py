#!/usr/bin/env python3
"""
Upsample /jackal_velocity_controller/odom (≈50 Hz) to TF at 200 Hz
without ever stamping transforms in the past or the future.
"""

import rospy, tf2_ros, tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

_ODOM_TOPIC  = "/jackal_velocity_controller/odom"
_ODOM_FRAME  = "odom"
_BASE_FRAME  = "base_link"
_RATE_HZ     = 200.0                       # desired TF rate
_MODE        = "predict"                   # "predict" or "repeat"

_last_odom = None
br         = tf2_ros.TransformBroadcaster()

def odom_cb(msg):
    global _last_odom
    # Drop stale odometry (can happen when a bag loops)
    if _last_odom and msg.header.stamp <= _last_odom.header.stamp:
        return
    _last_odom = msg

def publish_cb(event):
    if _last_odom is None:
        return

    now = rospy.Time.now()                 # *monotonic* ROS time
    dt  = (now - _last_odom.header.stamp).to_sec()

    p  = _last_odom.pose.pose.position
    q  = _last_odom.pose.pose.orientation
    vx = _last_odom.twist.twist

    # --- pose extrapolation -------------------------------------------------
    if _MODE == "predict" and dt > 0.0:
        # Linear
        px = p.x + vx.linear.x  * dt
        py = p.y + vx.linear.y  * dt
        pz = p.z + vx.linear.z  * dt
        # Angular
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))
        roll  += vx.angular.x * dt
        pitch += vx.angular.y * dt
        yaw   += vx.angular.z * dt
        qx, qy, qz, qw = tf_conversions.transformations.quaternion_from_euler(
            roll, pitch, yaw)
    else:
        px, py, pz     = p.x, p.y, p.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # --- broadcast ----------------------------------------------------------
    t                     = TransformStamped()
    t.header.stamp        = now
    t.header.frame_id     = _ODOM_FRAME
    t.child_frame_id      = _BASE_FRAME
    t.transform.translation.x = px
    t.transform.translation.y = py
    t.transform.translation.z = pz
    t.transform.rotation.x    = qx
    t.transform.rotation.y    = qy
    t.transform.rotation.z    = qz
    t.transform.rotation.w    = qw
    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf_upsampled")

    # Optional: still wait for /clock ≠ 0 if using sim time
    if rospy.get_param("/use_sim_time", False):
        while rospy.Time.now() == rospy.Time():
            rospy.sleep(0.05)

    rospy.Subscriber(_ODOM_TOPIC, Odometry, odom_cb, queue_size=50)
    rospy.Timer(rospy.Duration(1.0 / _RATE_HZ), publish_cb)

    rospy.loginfo(f"Upsampling {_ODOM_TOPIC} ({_MODE}) → TF {_RATE_HZ} Hz (monotonic)")
    rospy.spin()
