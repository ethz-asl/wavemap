#include <memory>
#include <utility>
#include <vector>

// wavemap_ros
#include "wavemap_ros/wavemap_node.hpp"

// wavemap
#include "wavemap/core/include/wavemap/tmp_check_3rdparty.hh"

// ROS 2 headers
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wavemap_ros {
MyCoreClass::MyCoreClass(const rclcpp::NodeOptions &options)
    : rclcpp::Node("wavemap_node", options) {
  int sum = wavemap::core::add_int(5, 5);
  RCLCPP_INFO_STREAM(get_logger(), "5 + 5 = " << sum);
  wavemap::core::check_3rdparty();
}
}  // namespace wavemap_ros
