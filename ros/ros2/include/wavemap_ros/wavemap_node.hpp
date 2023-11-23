#pragma once

// wavemap
#include "wavemap/core/include/wavemap/tmp_check_3rdparty.hh"

// ROS
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace wavemap_ros {

class MyCoreClass : public rclcpp::Node {
 public:
  MyCoreClass() = delete;
  explicit MyCoreClass(const rclcpp::NodeOptions &options);
};

}  // namespace wavemap_ros

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(wavemap_ros::MyCoreClass)