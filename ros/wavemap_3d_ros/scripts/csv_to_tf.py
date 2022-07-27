#!/usr/bin/env python3
import pandas
import numpy as np
import rospy
import tf


class StampedTransform:
    def __init__(self, csv_row):
        self.stamp = rospy.Time(secs=int(csv_row['#sec']),
                                nsecs=int(csv_row['nsec']))
        self.translation = np.array([csv_row['x'], csv_row['y'], csv_row['z']])
        self.rotation = np.array(
            [csv_row['qx'], csv_row['qy'], csv_row['qz'], csv_row['qw']])


# Params
update_rate = 100
world_frame = 'odom'
sensor_frame = 'cam_left'
csv_path = '../data/newer_college/registered_poses.csv'

# Setup ROS
rospy.init_node("csv_to_tf_node", anonymous=True)
rate = rospy.Rate(update_rate)
tf_broadcaster = tf.TransformBroadcaster()

# Load the CSV file
pose_dataframe = pandas.read_csv(csv_path)

# Wait until the rosbag starts playing (by checking if Clock is being published)
while rospy.get_time() == 0:
    rate.sleep()
start_time = rospy.get_time()

# Publish each pose when appropriate
current_row = 0
current_pose_stamped = StampedTransform(pose_dataframe.iloc[current_row])
while not rospy.is_shutdown():
    if current_pose_stamped.stamp + rospy.Duration(
            0, 10000000) < rospy.Time.now():
        tf_broadcaster.sendTransform(current_pose_stamped.translation,
                                     current_pose_stamped.rotation,
                                     current_pose_stamped.stamp, sensor_frame,
                                     world_frame)
        current_row += 1
        if len(pose_dataframe.index) < current_row:
            break
        current_pose_stamped = StampedTransform(
            pose_dataframe.iloc[current_row])
    rate.sleep()
