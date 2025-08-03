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
_MODE       = "predict"                        # "repeat" or "predict"

_last_odom   = None
_last_stamp  = None
br           = None

_RATE_HZ   = 200.0                     # desired TF rate
_PERIOD    = 1.0 / _RATE_HZ

_next_stamp = None                     # first stamp to be used
_last_wall  = None                     # wall-clock time of previous timer tick

def odom_cb(msg: Odometry):
    """Remember the newest odom sample *and* reset the interpolation."""
    global _last_odom, _last_stamp, _next_stamp, _last_wall
    _last_odom   = msg
    _last_stamp  = msg.header.stamp            # time in simulated clock
    _next_stamp  = _last_stamp                 # start interpolation here
    _last_wall   = rospy.get_time()            # wall-clock anchor

def timer_cb(event):
    """Run at 200 Hz, integrate pose, forge a unique stamp and broadcast."""
    global _next_stamp, _last_wall
    if _last_odom is None or _next_stamp is None:
        return                                 # no odom yet

    # ----- build the *next* stamp ------------------------------------------
    wall_now = rospy.get_time()
    steps    = int((wall_now - _last_wall) / _PERIOD) or 1
    _next_stamp += rospy.Duration.from_sec(steps * _PERIOD)
    _last_wall   = wall_now

    # ----- pose prediction (unchanged) -------------------------------------
    p = _last_odom.pose.pose.position
    q = _last_odom.pose.pose.orientation
    if _MODE == "predict":
        dt = (_next_stamp - _last_stamp).to_sec()
        v  = _last_odom.twist.twist
        px = p.x + v.linear.x  * dt
        py = p.y + v.linear.y  * dt
        pz = p.z + v.linear.z  * dt

        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))
        roll  += v.angular.x * dt
        pitch += v.angular.y * dt
        yaw   += v.angular.z * dt
        qx, qy, qz, qw = tf_conversions.transformations.quaternion_from_euler(
            roll, pitch, yaw)
    else:                                    # "repeat"
        px, py, pz   = p.x, p.y, p.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # ----- broadcast --------------------------------------------------------
    t = TransformStamped()
    t.header.stamp    = _next_stamp           # *** unique every tick ***
    t.header.frame_id = _ODOM_FRAME
    t.child_frame_id  = _BASE_FRAME
    t.transform.translation = Vector3(px, py, pz)
    t.transform.rotation    = Quaternion(qx, qy, qz, qw)
    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf_upsampled")

    if not rospy.get_param("/use_sim_time", False):
        rospy.logwarn(
            "Running on wall clock. For rosbag playback do:\n"
            "  rosparam set /use_sim_time true\n"
            "  rosbag play --clock your.bag")
    else:
        # Wait until /clock has started publishing (time != 0)
        while rospy.Time.now() == rospy.Time():
            rospy.sleep(0.05)

    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber(_ODOM_TOPIC, Odometry, odom_cb, queue_size=50)
    rospy.Timer(rospy.Duration(1.0 / _RATE_HZ), timer_cb)

    rospy.loginfo(f"Upsampling {_ODOM_TOPIC} ({_MODE}) → TF {_RATE_HZ} Hz")
    rospy.spin()
