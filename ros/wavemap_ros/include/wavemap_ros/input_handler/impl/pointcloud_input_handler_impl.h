#ifndef WAVEMAP_ROS_INPUT_HANDLER_IMPL_POINTCLOUD_INPUT_HANDLER_IMPL_H_
#define WAVEMAP_ROS_INPUT_HANDLER_IMPL_POINTCLOUD_INPUT_HANDLER_IMPL_H_

namespace wavemap {
template <typename RegistrarT>
bool PointcloudInputHandler::registerCallback(PointcloudTopicType type,
                                              RegistrarT registrar) {
  switch (type.toTypeId()) {
    case PointcloudTopicType::kPointCloud2:
      // clang-format off
      registrar(static_cast<void(PointcloudInputHandler::*)(
                    const sensor_msgs::PointCloud2&)>(
          &PointcloudInputHandler::callback));
      // clang-format on
      return true;
    case PointcloudTopicType::kOuster:
      // clang-format off
      registrar(static_cast<void(PointcloudInputHandler::*)(
                    const sensor_msgs::PointCloud2&)>(
          &PointcloudInputHandler::callback));
      // clang-format on
      return true;
    case PointcloudTopicType::kLivox:
#ifdef LIVOX_AVAILABLE
      // clang-format off
      registrar(static_cast<void(PointcloudInputHandler::*)(
                    const livox_ros_driver2::CustomMsg&)>(
          &PointcloudInputHandler::callback));
      // clang-format on
      return true;
#else
      LOG(ERROR) << "Livox support is currently not available. Please install "
                    "livox_ros_driver2 and rebuild wavemap.";
      return false;
#endif
    default:
      LOG(ERROR) << "Requested callback registration for unknown "
                    "PointcloudTopicType \""
                 << type.toStr() << "\"";
      return false;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_IMPL_POINTCLOUD_INPUT_HANDLER_IMPL_H_
