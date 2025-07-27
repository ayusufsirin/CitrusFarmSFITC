#!/usr/bin/env python3
"""
Convert a nav_msgs/Odometry topic to a TF transform.

 * subscribes: /jackal_velocity_controller/odom
 * publishes : TF (frame_id -> child_frame_id)
Edit the _ODOM_TOPIC, _ODOM_FRAME, _BASE_FRAME if your names differ.
"""
import rospy, tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

_ODOM_TOPIC  = "/jackal_velocity_controller/odom"
_ODOM_FRAME  = "odom"        # header.frame_id expected in the message
_BASE_FRAME  = "base_link"   # child_frame_id expected (or will be forced)

def cb(msg):
    t = TransformStamped()
    t.header            = msg.header
    t.header.frame_id   = _ODOM_FRAME          # enforce consistent name
    t.child_frame_id    = _BASE_FRAME
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation     = msg.pose.pose.orientation
    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf_bridge")
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber(_ODOM_TOPIC, Odometry, cb, queue_size=50)
    rospy.loginfo(f"Bridging {_ODOM_TOPIC} → TF [{_ODOM_FRAME}→{_BASE_FRAME}]")
    rospy.spin()
