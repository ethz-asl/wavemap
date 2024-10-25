#include <string>

#include <ros/ros.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

int main(int argc, char** argv) {
  // Register your node with ROS
  ros::init(argc, argv, "your_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create an empty map for illustration
  wavemap::HashedWaveletOctreeConfig config;
  wavemap::HashedWaveletOctree map(config);

  // Advertise the ROS topic
  const std::string ros_topic = "map";
  const int queue_size = 1;
  ros::Publisher map_pub =
      nh_private.advertise<wavemap_msgs::Map>(ros_topic, queue_size);

  // Convert the map to a ROS msg
  wavemap_msgs::Map map_msg;
  const std::string map_frame = "odom";
  const ros::Time stamp = ros::Time::now();
  wavemap::convert::mapToRosMsg(map, map_frame, stamp, map_msg);

  // Publish the ROS message
  map_pub.publish(map_msg);
}
