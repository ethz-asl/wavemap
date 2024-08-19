#ifndef WAVEMAP_ROS_INPUTS_ROS_INPUT_FACTORY_H_
#define WAVEMAP_ROS_INPUTS_ROS_INPUT_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap/core/utils/thread_pool.h"
#include "wavemap_ros/inputs/ros_input_base.h"

namespace wavemap {
class RosInputFactory {
 public:
  static std::unique_ptr<RosInputBase> create(
      const param::Value& params, std::shared_ptr<Pipeline> pipeline,
      std::shared_ptr<TfTransformer> transformer, std::string world_frame,
      ros::NodeHandle nh, ros::NodeHandle nh_private);

  static std::unique_ptr<RosInputBase> create(
      RosInputType input_type, const param::Value& params,
      std::shared_ptr<Pipeline> pipeline,
      std::shared_ptr<TfTransformer> transformer, std::string world_frame,
      ros::NodeHandle nh, ros::NodeHandle nh_private);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_ROS_INPUT_FACTORY_H_
