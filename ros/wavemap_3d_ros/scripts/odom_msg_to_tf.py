#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def callback_odometry(odom_msg):
    pos_msg = odom_msg.pose.pose.position
    ori_msg = odom_msg.pose.pose.orientation
    position = (pos_msg.x, pos_msg.y, pos_msg.z)
    orientation = (ori_msg.x, ori_msg.y, ori_msg.z, ori_msg.w)
    tf_broadcaster.sendTransform(position, orientation, odom_msg.header.stamp,
                                 sensor_frame, world_frame)


def callback_pose_stamped(odom_msg):
    pos_msg = odom_msg.pose.position
    ori_msg = odom_msg.pose.orientation
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
topic = rospy.get_param('~topic')
topic_type = rospy.get_param('~topic_type')

# Run the republisher
if topic_type == "Odometry":
    rospy.Subscriber(topic, Odometry, callback_odometry)
if topic_type == "PoseStamped":
    rospy.Subscriber(topic, PoseStamped, callback_pose_stamped)

rospy.spin()
