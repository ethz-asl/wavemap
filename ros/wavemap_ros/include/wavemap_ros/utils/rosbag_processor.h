#ifndef WAVEMAP_ROS_UTILS_ROSBAG_PROCESSOR_H_
#define WAVEMAP_ROS_UTILS_ROSBAG_PROCESSOR_H_

#include <functional>
#include <list>
#include <map>
#include <string>

#include <glog/logging.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

namespace wavemap {
class RosbagProcessor {
 public:
  RosbagProcessor() = default;
  ~RosbagProcessor();

  void addRosbag(const std::string& rosbag_path);
  bool addRosbags(std::istringstream& rosbag_paths);
  bool bagsContainTopic(const std::string& topic_name);

  template <typename MessageT>
  void addCallback(const std::string& ros_topic_name,
                   std::function<void(MessageT)> function_ptr);

  template <typename MessageT, typename CallbackObjectT>
  void addCallback(const std::string& ros_topic_name,
                   void (CallbackObjectT::*function_ptr)(MessageT),
                   CallbackObjectT* object_ptr);

  template <typename MessageT>
  void addRepublisher(const std::string& rosbag_topic_name,
                      const std::string& republished_topic_name,
                      ros::NodeHandle& nh, unsigned int queue_size);

  void enableSimulatedClock(ros::NodeHandle nh) {
    simulated_clock_pub_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  }
  void disableSimulatedClock() { simulated_clock_pub_.reset(); }

  bool processAll();

 private:
  std::list<rosbag::Bag> opened_rosbags_;
  rosbag::View bag_view_;

  using RosbagCallback = std::function<void(const rosbag::MessageInstance&)>;
  std::map<std::string, RosbagCallback, std::less<>> callbacks_;
  std::map<std::string, ros::Publisher> republishers_;
  std::optional<ros::Publisher> simulated_clock_pub_;
};
}  // namespace wavemap

#include "wavemap_ros/impl/rosbag_processor_inl.h"

#endif  // WAVEMAP_ROS_UTILS_ROSBAG_PROCESSOR_H_
