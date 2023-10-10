#include <ros/ros.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

// Create a smart pointer that will own the received map
wavemap::VolumetricDataStructureBase::Ptr loaded_map;

// Define the map callback
void mapCallback(const wavemap_msgs::Map& msg) {
  // Load the received map
  wavemap::convert::rosMsgToMap(msg, loaded_map);
}

// NOTE: We only define loaded_map and mapCallback globally for illustration
//       purposes. In practice, we recommend defining them as class members.

int main(int argc, char** argv) {
  // Register your node with ROS
  ros::init(argc, argv, "your_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Subscribe to the ROS topic
  const std::string ros_topic = "map";
  const int queue_size = 1;
  ros::Subscriber map_sub = nh.subscribe(ros_topic, queue_size, &mapCallback);
}
