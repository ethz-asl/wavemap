#include <cmath>
#include <iostream>

#include <ros/time.h>
#include <wavemap_ros_conversions/layered_map_msg_conversions.h>

#include "../common/field_layered_map_ros_config.h"

namespace field_map = wavemap::examples::field_map;
namespace field_map_ros = wavemap::examples::field_map::ros_config;

namespace {
bool approximatelyEqual(float lhs, float rhs, float tolerance = 1e-5f) {
  return std::abs(lhs - rhs) <= tolerance;
}
}  // namespace

int main() {
  field_map::Map original(field_map::makeDefaultConfig());
  const wavemap::Index3D index(1, 2, 3);
  original.continuousMap().setVoxelValue(
      index, field_map::Voxel(
                 0.8f, field_map::makeContinuousLayers(0.7f)));
  original.discreteLayers().get<field_map::ClassLayer>().setValue(
      index, static_cast<int>(field_map::ClassLabel::kObstacle));

  wavemap_msgs::LayeredMap message;
  if (!wavemap::convert::layeredMapToRosMsg<
          field_map::Map, field_map_ros::VoxelRosConverter>(
          original, "map", ros::Time(10), message)) {
    return 1;
  }

  field_map::Map reconstructed(field_map::makeDefaultConfig());
  if (!wavemap::convert::rosMsgToLayeredMap<
          field_map::Map, field_map_ros::VoxelRosConverter>(
          message, reconstructed)) {
    return 2;
  }

  const auto voxel = reconstructed.continuousMap().getVoxelValue(index);
  const auto class_value =
      reconstructed.discreteLayers().get<field_map::ClassLayer>().getValue(index);
  const bool ok =
      message.header.frame_id == "map" &&
      message.continuous_map.layered_hashed_wavelet_octree.size() == 1u &&
      approximatelyEqual(voxel.data.get<field_map::ReflectivityLayer>(), 0.7f) &&
      class_value &&
      *class_value == static_cast<int>(field_map::ClassLabel::kObstacle);

  std::cout << "Field LayeredMap ROS round trip: "
            << (ok ? "pass" : "FAIL") << "\n";
  return ok ? 0 : 3;
}
