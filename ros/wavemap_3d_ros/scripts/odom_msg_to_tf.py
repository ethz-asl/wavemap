#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry


def callback(odom_msg):
    pos_msg = odom_msg.pose.pose.position
    ori_msg = odom_msg.pose.pose.orientation
    position = (pos_msg.x, pos_msg.y, pos_msg.z)
    orientation = (ori_msg.x, ori_msg.y, ori_msg.z, ori_msg.w)
    tf_broadcaster.sendTransform(position, orientation, odom_msg.header.stamp,
                                 sensor_frame, world_frame)


# Setup ROS
rospy.init_node("odom_msg_to_tf_node", anonymous=True)
tf_broadcaster = tf.TransformBroadcaster()

# Params
world_frame = rospy.get_param('~odom_frame', 'odom')
sensor_frame = rospy.get_param('~sensor_frame')
odom_topic = rospy.get_param('~odom_topic')

# Run the republisher
rospy.Subscriber(odom_topic, Odometry, callback)
rospy.spin()
