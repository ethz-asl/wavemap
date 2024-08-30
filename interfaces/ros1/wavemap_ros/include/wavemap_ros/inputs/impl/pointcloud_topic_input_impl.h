#ifndef WAVEMAP_ROS_INPUTS_IMPL_POINTCLOUD_TOPIC_INPUT_IMPL_H_
#define WAVEMAP_ROS_INPUTS_IMPL_POINTCLOUD_TOPIC_INPUT_IMPL_H_

namespace wavemap {
template <typename RegistrarT>
bool PointcloudTopicInput::registerCallback(PointcloudTopicType type,
                                            RegistrarT registrar) {
  switch (type) {
    case PointcloudTopicType::kPointCloud2:
    case PointcloudTopicType::kOuster:
      // clang-format off
      registrar(static_cast<void(PointcloudTopicInput::*)(
                    const sensor_msgs::PointCloud2&)>(
          &PointcloudTopicInput::callback));
      // clang-format on
      return true;
    case PointcloudTopicType::kLivox:
#ifdef LIVOX_AVAILABLE
      // clang-format off
      registrar(static_cast<void(PointcloudTopicInput::*)(
                    const livox_ros_driver2::CustomMsg&)>(
          &PointcloudTopicInput::callback));
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

#endif  // WAVEMAP_ROS_INPUTS_IMPL_POINTCLOUD_TOPIC_INPUT_IMPL_H_
