#include "wavemap_ros/rosbag_processor.h"

namespace wavemap {
RosbagProcessor::~RosbagProcessor() {
  ROS_INFO("Shutting down...");
  for (rosbag::Bag& opened_rosbag : opened_rosbags_) {
    opened_rosbag.close();
  }
}

void RosbagProcessor::addRosbag(const std::string& rosbag_path) {
  opened_rosbags_.emplace_back(rosbag_path);
  bag_view_.addQuery(opened_rosbags_.back());
  LOG(INFO) << "Loaded rosbag " << rosbag_path;
}

bool RosbagProcessor::addRosbags(std::istringstream& rosbag_paths) {
  std::string rosbag_path;
  while (rosbag_paths >> rosbag_path) {
    addRosbag(rosbag_path);
    if (!ros::ok()) {
      return false;
    }
  }
  return true;
}

bool RosbagProcessor::bagsContainTopic(const std::string& topic_name) {
  const auto connections = bag_view_.getConnections();
  return std::any_of(connections.cbegin(), connections.cend(),
                     [&](const rosbag::ConnectionInfo* info) {
                       return info && info->topic == topic_name;
                     });
}

bool RosbagProcessor::processAll() {
  ros::Time side_tasks_last_timestamp(0);
  const ros::Duration kSideTasksDt(0.01);

  // Give the subscribers and rest of the system some time to set up
  for (int idx = 0; idx < 5; ++idx) {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }

  for (const rosbag::MessageInstance& msg : bag_view_) {
    // Exit if CTRL+C was pressed
    if (!ros::ok()) {
      return false;
    }

    // Handle callbacks
    if (auto it = callbacks_.find(msg.getTopic()); it != callbacks_.end()) {
      it->second(msg);
    }

    // Handle republishing
    if (auto it = republishers_.find(msg.getTopic());
        it != republishers_.end()) {
      it->second.publish(msg);
    }

    // Catch up on some periodic tasks
    if (msg.getTime() < side_tasks_last_timestamp ||
        side_tasks_last_timestamp + kSideTasksDt < msg.getTime()) {
      side_tasks_last_timestamp = msg.getTime();

      // Publish clock substitute if needed
      if (simulated_clock_pub_.has_value()) {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = msg.getTime();
        simulated_clock_pub_.value().publish(clock_msg);
      }

      // Process node callbacks (publishers, timers,...)
      ros::spinOnce();
    }
  }

  return true;
}
}  // namespace wavemap
