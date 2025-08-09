#!/usr/bin/env python3
import rospy, tf2_ros, tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

_ODOM_TOPIC  = "/jackal_velocity_controller/odom"
_ODOM_FRAME  = "odom"
_BASE_FRAME  = "base_link"
_RATE_HZ     = 200.0
_MODE        = "predict"   # "predict" or "repeat"

_last_odom = None
_last_now  = None
br         = tf2_ros.TransformBroadcaster()

def odom_cb(msg):
    global _last_odom
    _last_odom = msg

def _safe_sleep(rate):
    try:
        rate.sleep()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        pass

if __name__ == "__main__":
    rospy.init_node("odom_to_tf_upsampled")

    use_sim_time = rospy.get_param("/use_sim_time", False)
    if use_sim_time:
        while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time():
            rospy.sleep(0.05)

    rospy.Subscriber(_ODOM_TOPIC, Odometry, odom_cb, queue_size=100)
    rate = rospy.Rate(_RATE_HZ)
    rospy.loginfo(f"Upsampling {_ODOM_TOPIC} ({_MODE}) → TF {_RATE_HZ} Hz (bag-safe)")

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # handle time rewind (bag loop/restart)
        if _last_now is not None and now < _last_now:
            rospy.logwarn_throttle(2.0, "ROS time moved backwards; resetting state.")
            _last_odom = None
            _last_now  = now
            _safe_sleep(rate)
            continue
        _last_now = now

        if _last_odom is None:
            _safe_sleep(rate)
            continue

        # never publish in the future
        if _last_odom.header.stamp > now:
            _safe_sleep(rate)
            continue

        dt = max(0.0, (now - _last_odom.header.stamp).to_sec())

        p = _last_odom.pose.pose.position
        q = _last_odom.pose.pose.orientation
        v = _last_odom.twist.twist

        if _MODE == "predict" and dt > 0.0:
            px = p.x + v.linear.x * dt
            py = p.y + v.linear.y * dt
            pz = p.z + v.linear.z * dt
            r, pit, yaw = tf_conversions.transformations.euler_from_quaternion(
                (q.x, q.y, q.z, q.w))
            r   += v.angular.x * dt
            pit += v.angular.y * dt
            yaw += v.angular.z * dt
            qx, qy, qz, qw = tf_conversions.transformations.quaternion_from_euler(r, pit, yaw)
        else:
            px, py, pz = p.x, p.y, p.z
            qx, qy, qz, qw = q.x, q.y, q.z, q.w

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = _ODOM_FRAME
        t.child_frame_id  = _BASE_FRAME
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation.x    = qx
        t.transform.rotation.y    = qy
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        br.sendTransform(t)

        _safe_sleep(rate)
