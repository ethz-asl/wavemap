#ifndef WAVEMAP_3D_ROS_INPUT_HANDLER_INPUT_HANDLER_FACTORY_H_
#define WAVEMAP_3D_ROS_INPUT_HANDLER_INPUT_HANDLER_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap_3d_ros/input_handler/input_handler.h"

namespace wavemap {
class InputHandlerFactory {
 public:
  static std::unique_ptr<InputHandler> create(
      const param::Map& integrator_params, std::string world_frame,
      VolumetricDataStructure3D::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer, const ros::NodeHandle& nh,
      std::optional<InputHandlerType> default_input_handler_type =
          std::nullopt);

  static std::unique_ptr<InputHandler> create(
      InputHandlerType input_handler_type, const param::Map& integrator_params,
      std::string world_frame, VolumetricDataStructure3D::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer, const ros::NodeHandle& nh);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_ROS_INPUT_HANDLER_INPUT_HANDLER_FACTORY_H_
