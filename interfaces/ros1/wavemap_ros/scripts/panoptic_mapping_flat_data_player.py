#!/usr/bin/env python3

# All credits go to:
# https://github.com/ethz-asl/panoptic_mapping

import os
import csv

from copy import deepcopy
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from PIL import Image as PilImage
import numpy as np
import tf

from std_srvs.srv import Empty, EmptyResponse


class FlatDataPlayer():
    # pylint: disable=too-many-instance-attributes
    def __init__(self):
        """  Initialize ros node and read params """
        # params
        self.data_path = rospy.get_param('~data_path', '')
        self.global_frame_name = rospy.get_param('~global_frame_name', 'world')
        self.sensor_frame_name = rospy.get_param('~sensor_frame_name',
                                                 "depth_cam")
        self.play_rate = rospy.get_param('~play_rate', 1.0)
        self.wait = rospy.get_param('~wait', False)
        self.max_frames = rospy.get_param('~max_frames', 1000000000)
        self.refresh_rate = 100  # Hz

        # ROS
        self.color_pub = rospy.Publisher("~color_image", Image, queue_size=100)
        self.color_info_pub = rospy.Publisher("~color_image/camera_info",
                                              CameraInfo,
                                              queue_size=100)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=100)
        self.depth_info_pub = rospy.Publisher("~depth_image/camera_info",
                                              CameraInfo,
                                              queue_size=100)
        self.id_pub = rospy.Publisher("~id_image", Image, queue_size=100)
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=100)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # setup
        self.cv_bridge = CvBridge()
        stamps_file = os.path.join(self.data_path, 'timestamps.csv')
        self.times = []
        self.ids = []
        self.current_index = 0  # Used to iterate through
        if not os.path.isfile(stamps_file):
            rospy.logfatal(f"No timestamp file '{stamps_file}' found.")
        with open(stamps_file, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.ids.append(str(row[0]))
                self.times.append(float(row[1]) / 1e9)

        # Populate the camera_info messages, written out in intrinsics.txt
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = 640
        self.camera_info_msg.height = 480
        self.camera_info_msg.K[0] = 320  # fx
        self.camera_info_msg.K[4] = 320  # fy
        self.camera_info_msg.K[2] = 320  # cx
        self.camera_info_msg.K[5] = 240  # cy

        self.ids = [x for _, x in sorted(zip(self.times, self.ids))]
        self.times = sorted(self.times)
        self.times = [(x - self.times[0]) / self.play_rate for x in self.times]
        self.start_time = None

        if self.wait:
            self.start_srv = rospy.Service('~start', Empty, self.start)
        else:
            self.start(None)

    def start(self, _):
        self.running = True
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.refresh_rate),
                                 self.callback)
        return EmptyResponse()

    def callback(self, _):
        # Check we should be publishing.
        if not self.running:
            return

        # Check we're not done.
        if self.current_index >= len(self.times):
            rospy.loginfo("Finished playing the dataset.")
            rospy.signal_shutdown("Finished playing the dataset.")
            return

        # Check the time.
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now
        if self.times[self.current_index] > (now - self.start_time).to_sec():
            return

        # Get all data and publish.
        file_id = os.path.join(self.data_path, self.ids[self.current_index])

        # Color.
        color_file = file_id + "_color.png"
        depth_file = file_id + "_depth.tiff"
        pose_file = file_id + "_pose.txt"
        files = [color_file, depth_file, pose_file]
        for f in files:
            if not os.path.isfile(f):
                rospy.logwarn(f"Could not find file '{f}', skipping frame.")
                self.current_index += 1
                return

        # Load and publish Color image.
        cv_img = cv2.imread(color_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.color_pub.publish(img_msg)

        # Load and publish ID image.
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img[:, :, 0], "8UC1")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.id_pub.publish(img_msg)

        # Load and publish depth image. These are optional.
        cv_img = PilImage.open(depth_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(np.array(cv_img), "32FC1")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.depth_pub.publish(img_msg)

        # Publish the camera info messages
        cam_info_msg = CameraInfo()
        cam_info_msg = deepcopy(self.camera_info_msg)
        cam_info_msg.header = img_msg.header
        self.depth_info_pub.publish(cam_info_msg)
        self.color_info_pub.publish(cam_info_msg)

        # Load and publish transform.
        if os.path.isfile(pose_file):
            with open(pose_file, 'r') as f:
                pose_data = [float(x) for x in f.read().split()]
                transform = np.eye(4)
                for row in range(4):
                    for col in range(4):
                        transform[row, col] = pose_data[row * 4 + col]
                rotation = tf.transformations.quaternion_from_matrix(transform)
                self.tf_broadcaster.sendTransform(
                    (transform[0, 3], transform[1, 3], transform[2, 3]),
                    rotation, now, self.sensor_frame_name,
                    self.global_frame_name)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.global_frame_name
        pose_msg.pose.position.x = pose_data[3]
        pose_msg.pose.position.y = pose_data[7]
        pose_msg.pose.position.z = pose_data[11]
        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]
        self.pose_pub.publish(pose_msg)

        self.current_index += 1
        if self.current_index > self.max_frames:
            rospy.signal_shutdown(
                f"Played reached max frames ({self.max_frames})")


if __name__ == '__main__':
    rospy.init_node('flat_data_player')
    flat_data_player = FlatDataPlayer()
    rospy.spin()
