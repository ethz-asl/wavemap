#ifndef WAVEMAP_ROS_INPUTS_INPUT_FACTORY_H_
#define WAVEMAP_ROS_INPUTS_INPUT_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap/utils/thread_pool.h"
#include "wavemap_ros/inputs/input_base.h"

namespace wavemap {
class InputFactory {
 public:
  static std::unique_ptr<InputBase> create(
      const param::Value& params, std::string world_frame,
      VolumetricDataStructureBase::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer,
      std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh,
      ros::NodeHandle nh_private,
      std::optional<InputType> default_input_type = std::nullopt,
      std::function<void(const ros::Time&)> map_update_callback = {});

  static std::unique_ptr<InputBase> create(
      InputType input_type, const param::Value& params, std::string world_frame,
      VolumetricDataStructureBase::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer,
      std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh,
      ros::NodeHandle nh_private,
      std::function<void(const ros::Time&)> map_update_callback = {});
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_INPUT_FACTORY_H_
