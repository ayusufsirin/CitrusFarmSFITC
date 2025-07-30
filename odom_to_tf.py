#!/usr/bin/env python3
"""
Upsample an Odometry stream (≈50 Hz) to a TF transform at 200 Hz.

  subscribes : /jackal_velocity_controller/odom  (50 Hz)
  publishes  : TF (odom -> base_link)            (200 Hz)
"""

import rospy, tf2_ros
import tf_conversions                         # for quaternion maths
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

_ODOM_TOPIC  = "/jackal_velocity_controller/odom"
_ODOM_FRAME  = "odom"
_BASE_FRAME  = "base_link"
_RATE_HZ     = 200                             # desired TF rate
_MODE       = "predict"                        # "repeat" or "predict"

_last_odom   = None
_last_stamp  = None
br           = None

def odom_cb(msg: Odometry):
    global _last_odom, _last_stamp
    _last_odom  = msg
    _last_stamp = msg.header.stamp

def timer_cb(event):
    """Called every 1/_RATE_HZ s – re‑broadcast or predict a transform."""
    if _last_odom is None:
        return                                  # nothing received yet

    now = rospy.Time.now()

    # Pose at last odom sample
    p = _last_odom.pose.pose.position
    q = _last_odom.pose.pose.orientation

    if _MODE == "predict":
        dt = (now - _last_stamp).to_sec()
        v  = _last_odom.twist.twist            # linear & angular vel.

        # -------- Constant‑velocity integration --------
        # linear:
        px = p.x + v.linear.x * dt
        py = p.y + v.linear.y * dt
        pz = p.z + v.linear.z * dt

        # angular (integrate quaternion derivative):
        # Convert quaternion to RPY → add angular*dt → back to quat
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))
        roll  += v.angular.x * dt
        pitch += v.angular.y * dt
        yaw   += v.angular.z * dt
        qx, qy, qz, qw = tf_conversions.transformations.quaternion_from_euler(
            roll, pitch, yaw)
    else:                                       # "repeat"
        px, py, pz = p.x, p.y, p.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # -------- Publish TF --------
    t = TransformStamped()
    t.header.stamp    = now
    t.header.frame_id = _ODOM_FRAME
    t.child_frame_id  = _BASE_FRAME
    t.transform.translation = Vector3(px, py, pz)
    t.transform.rotation    = Quaternion(qx, qy, qz, qw)

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf_upsampled")
    rospy.set_param('/use_sim_time', True)

    br = tf2_ros.TransformBroadcaster()

    rospy.Subscriber(_ODOM_TOPIC, Odometry, odom_cb, queue_size=50)
    rospy.Timer(rospy.Duration(1.0/_RATE_HZ), timer_cb)

    rospy.loginfo(f"Upsampling {_ODOM_TOPIC} ({_MODE}) → TF {_RATE_HZ} Hz")
    rospy.spin()
