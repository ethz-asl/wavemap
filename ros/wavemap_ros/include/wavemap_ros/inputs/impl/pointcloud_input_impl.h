#ifndef WAVEMAP_ROS_INPUTS_IMPL_POINTCLOUD_INPUT_IMPL_H_
#define WAVEMAP_ROS_INPUTS_IMPL_POINTCLOUD_INPUT_IMPL_H_

namespace wavemap {
template <typename RegistrarT>
bool PointcloudInput::registerCallback(PointcloudTopicType type,
                                       RegistrarT registrar) {
  switch (type.toTypeId()) {
    case PointcloudTopicType::kPointCloud2:
    case PointcloudTopicType::kOuster:
      // clang-format off
      registrar(static_cast<void(PointcloudInput::*)(
                    const sensor_msgs::PointCloud2&)>(
          &PointcloudInput::callback));
      // clang-format on
      return true;
    case PointcloudTopicType::kLivox:
#ifdef LIVOX_AVAILABLE
      // clang-format off
      registrar(static_cast<void(PointcloudInput::*)(
                    const livox_ros_driver2::CustomMsg&)>(
          &PointcloudInput::callback));
      // clang-format on
      return true;
#else
      ROS_ERROR(
          "Livox support is currently not available. Please install "
          "livox_ros_driver2 and rebuild wavemap.");
      return false;
#endif
    default:
      ROS_ERROR_STREAM(
          "Requested callback registration for unknown PointcloudTopicType \""
          << type.toStr() << "\"");
      return false;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_IMPL_POINTCLOUD_INPUT_IMPL_H_
