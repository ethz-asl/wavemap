#include <cmath>
#include <iostream>
#include <string>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/layered/layered_pipeline.h>
#include <wavemap_msgs/LayeredMapUpdate.h>
#include <wavemap_ros_conversions/descriptor_layer_observation_conversions.h>

#include "../common/field_layered_map_ros_config.h"

namespace field_map = wavemap::examples::field_map;

namespace {
bool approximatelyEqual(float lhs, float rhs, float tolerance = 1e-5f) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool expect(const std::string& label, bool condition) {
  std::cout << "  " << label << ": " << (condition ? "pass" : "FAIL")
            << "\n";
  return condition;
}

geometry_msgs::Point32 point(float x, float y, float z) {
  geometry_msgs::Point32 result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}
}  // namespace

int main() {
  field_map::Map map(field_map::makeDefaultConfig());
  wavemap::layered::LayeredPipeline<field_map::Map> pipeline(map);

  wavemap_msgs::LayeredMapUpdate update;
  update.header.frame_id = "map";
  update.positions = {point(0.1f, 0.1f, 0.1f),
                      point(-0.4f, 0.2f, 0.6f)};

  wavemap_msgs::Layer reflectivity;
  reflectivity.name = "reflectivity";
  reflectivity.type = "float32";
  reflectivity.float32_values = {0.25f, 0.8f};
  update.layers.emplace_back(reflectivity);

  wavemap_msgs::Layer class_layer;
  class_layer.name = "class";
  class_layer.type = "int32";
  class_layer.int32_values = {
      static_cast<int>(field_map::ClassLabel::kGround),
      static_cast<int>(field_map::ClassLabel::kObstacle)};
  update.layers.emplace_back(class_layer);

  std::string error;
  const bool converted =
      wavemap::convert::DescriptorLayerObservationRosConverter<
          field_map::Map>::integrate(update, pipeline, &error);

  const float cell_width_inv =
      1.f / map.continuousMap().getMinCellWidth();
  const auto first_index = wavemap::convert::pointToNearestIndex(
      wavemap::Point3D(0.1f, 0.1f, 0.1f), cell_width_inv);
  const auto second_index = wavemap::convert::pointToNearestIndex(
      wavemap::Point3D(-0.4f, 0.2f, 0.6f), cell_width_inv);

  bool ok = true;
  std::cout << "Field layered ROS update Phase 4 experiment\n";
  ok &= expect("valid subset converted", converted && error.empty());
  ok &= expect(
      "continuous layer routed through pipeline",
      approximatelyEqual(
          map.continuousMap().getVoxelValue(first_index).data.get<field_map::ReflectivityLayer>(), 0.25f) &&
          approximatelyEqual(
              map.continuousMap().getVoxelValue(second_index).data.get<field_map::ReflectivityLayer>(),
              0.8f));
  const auto first_class =
      map.discreteLayers().get<field_map::ClassLayer>().getValue(first_index);
  const auto second_class =
      map.discreteLayers().get<field_map::ClassLayer>().getValue(second_index);
  ok &= expect(
      "discrete layer routed through pipeline",
      first_class && second_class &&
          *first_class ==
              static_cast<int>(field_map::ClassLabel::kGround) &&
          *second_class ==
              static_cast<int>(field_map::ClassLabel::kObstacle));

  wavemap_msgs::LayeredMapUpdate malformed = update;
  malformed.layers.front().float32_values = {0.9f};
  error.clear();
  const bool malformed_accepted =
      wavemap::convert::DescriptorLayerObservationRosConverter<
          field_map::Map>::integrate(malformed, pipeline, &error);
  ok &= expect(
      "malformed batch rejected without partial update",
      !malformed_accepted && !error.empty() &&
          approximatelyEqual(
              map.continuousMap().getVoxelValue(first_index).data.get<field_map::ReflectivityLayer>(),
              0.25f));

  wavemap_msgs::LayeredMapUpdate unknown = update;
  unknown.layers.front().name = "not_in_schema";
  error.clear();
  const bool unknown_accepted =
      wavemap::convert::DescriptorLayerObservationRosConverter<
          field_map::Map>::integrate(unknown, pipeline, &error);
  ok &= expect("unknown layer rejected",
               !unknown_accepted && !error.empty());

  return ok ? 0 : 1;
}
