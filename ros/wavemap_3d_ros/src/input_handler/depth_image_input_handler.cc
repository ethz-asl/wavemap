#include "wavemap_3d_ros/input_handler/depth_image_input_handler.h"

namespace wavemap {
DepthImageInputHandler::DepthImageInputHandler(
    const param::Map& params, std::string world_frame,
    VolumetricDataStructure3D::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh)
    : InputHandler(params, std::move(world_frame), std::move(occupancy_map),
                   std::move(transformer), nh) {
  // Subscribe to the depth image input
  depth_image_sub_ =
      nh.subscribe(config_.topic_name, config_.topic_queue_length,
                   &DepthImageInputHandler::depthImageCallback, this);
}

void DepthImageInputHandler::processQueue() {
  // TODO(victorr): Implement this
}
}  // namespace wavemap
