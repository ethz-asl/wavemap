#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/layered/layered_pipeline.h>

#include "../../common/field_layered_map_config.h"

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
}  // namespace

int main() {
  field_map::Map map(field_map::makeDefaultConfig());
  wavemap::layered::LayeredPipeline<field_map::Map> pipeline(map);

  const wavemap::Point3D point(0.7f, -0.1f, 0.8f);
  const wavemap::Index3D index = wavemap::convert::pointToNearestIndex(
      point, 1.f / map.continuousMap().getMinCellWidth());
  map.continuousMap().setVoxelValue(
      index, field_map::Voxel(
                 0.6f, field_map::makeContinuousLayers(0.2f)));

  const std::vector<wavemap::layered::LayerObservation<float>>
      reflectivity_observations{
          {point, 0.4f},
          {point, 0.65f},
          {wavemap::Point3D(
               std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f),
           0.9f}};
  const auto reflectivity_result =
      pipeline.integrate<field_map::ReflectivityLayer>(
          reflectivity_observations);

  const std::vector<wavemap::layered::LayerObservation<int>>
      class_observations{
          {point,
           static_cast<int>(field_map::ClassLabel::kGround)},
          {point,
           static_cast<int>(field_map::ClassLabel::kObstacle)}};
  const auto class_result =
      pipeline.integrate<field_map::ClassLayer>(class_observations);

  const auto voxel = map.continuousMap().getVoxelValue(index);
  const auto class_value =
      map.discreteLayers().get<field_map::ClassLayer>().getValue(index);

  bool ok = true;
  std::cout << "Field LayeredMap Phase 3 pipeline experiment\n";
  ok &= expect("world position converted to voxel index",
               reflectivity_result.updated_voxels == 1u);
  ok &= expect("continuous observation integrated",
               approximatelyEqual(voxel.data.get<field_map::ReflectivityLayer>(), 0.65f));
  ok &= expect("occupancy preserved",
               approximatelyEqual(voxel.occupancy, 0.6f));
  ok &= expect("invalid position rejected",
               reflectivity_result.received == 3u &&
                   reflectivity_result.integrated == 2u &&
                   reflectivity_result.rejected == 1u);
  ok &= expect(
      "discrete policy and storage updated",
      class_result.integrated == 2u && class_value &&
          *class_value ==
              static_cast<int>(field_map::ClassLabel::kObstacle));

  return ok ? 0 : 1;
}
