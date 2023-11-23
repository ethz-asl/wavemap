#include <memory>
#include <utility>
#include <vector>

// wavemap_ros
#include "wavemap_ros/wavemap_node.hpp"

// wavemap
#include "wavemap/core/include/wavemap/tmp_check_3rdparty.hh"

// ROS 1 headers
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

namespace wavemap_ros {
MyCoreClass::MyCoreClass(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {}
}  // namespace wavemap_ros

int main(int argc, char **argv) {
  ros::init(argc, argv, "wavemap_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  wavemap_ros::MyCoreClass node(nh, nh_private);
  int sum = wavemap::core::add_int(5, 5);
  ROS_INFO_STREAM("5 + 5 = " << sum);
  wavemap::core::check_3rdparty();

  ros::spin();

  return 0;
}