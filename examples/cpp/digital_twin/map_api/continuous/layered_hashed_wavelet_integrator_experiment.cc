#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/core/integrator/measurement_model/continuous_beam.h>
#include <wavemap/core/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h>
#include <wavemap/core/integrator/projection_model/spherical_projector.h>

#include "../../common/layered_voxel_config.h"

namespace {
bool approximatelyEqual(float lhs, float rhs, float tolerance = 1e-5f) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool approximatelyEqual(const Rgb& lhs, const Rgb& rhs,
                        float tolerance = 1e-5f) {
  return sumAbsDiff(lhs, rhs) <= tolerance;
}
}  // namespace

int main() {
  wavemap::HashedWaveletOctreeConfig map_config;
  map_config.min_cell_width = 0.1f;
  map_config.min_log_odds = -2.f;
  map_config.max_log_odds = 4.f;
  map_config.tree_height = 4;

  auto map = std::make_shared<ContinuousWaveletMap>(map_config);

  const wavemap::Index3D seeded_index(10, 0, 0);
  const LayeredVoxel seeded_voxel =
      makeLayeredVoxel(0.f, rgb(0.25f, 0.5f, 0.75f), 0.8f);
  map->setVoxelValue(seeded_index, seeded_voxel);

  auto projection_model = std::make_shared<wavemap::SphericalProjector>(
      wavemap::SphericalProjectorConfig(
          wavemap::CircularProjectorConfig(-1.2f, 1.2f, 32),
          wavemap::CircularProjectorConfig(-1.2f, 1.2f, 32)));
  auto posed_range_image =
      std::make_shared<wavemap::PosedImage<>>(projection_model->getDimensions());
  auto beam_offset_image =
      std::make_shared<wavemap::Image<wavemap::Vector2D>>(
          projection_model->getDimensions());

  wavemap::ContinuousBeamConfig measurement_config;
  measurement_config.angle_sigma = 0.01f;
  measurement_config.range_sigma = 0.02f;
  measurement_config.scaling_free = 0.2f;
  measurement_config.scaling_occupied = 0.4f;
  auto measurement_model = std::make_shared<wavemap::ContinuousBeam>(
      measurement_config, projection_model, posed_range_image,
      beam_offset_image);

  wavemap::ProjectiveIntegratorConfig integrator_config;
  integrator_config.min_range = 0.05f;
  integrator_config.max_range = 2.f;
  integrator_config.max_update_resolution = 0.f;
  integrator_config.termination_update_error = 0.05f;

  wavemap::HashedWaveletIntegratorT<LayeredVoxel> integrator(
      integrator_config, projection_model, posed_range_image,
      beam_offset_image, measurement_model, map);

  wavemap::Pointcloud<> pointcloud;
  pointcloud.resize(1u);
  pointcloud[0] = wavemap::Point3D(1.f, 0.f, 0.f);
  const wavemap::Transformation3D T_W_C;
  const wavemap::PosedPointcloud<> posed_pointcloud(T_W_C, pointcloud);

  const LayeredVoxel before = map->getVoxelValue(seeded_index);
  const size_t size_before = map->size();

  integrator.integrate(posed_pointcloud);
  map->threshold();

  const LayeredVoxel after = map->getVoxelValue(seeded_index);
  const size_t size_after = map->size();

  float max_abs_occupancy = 0.f;
  map->forEachVoxelLeaf([&max_abs_occupancy](const wavemap::OctreeIndex& /*index*/,
                                             const LayeredVoxel& voxel) {
    max_abs_occupancy = std::max(max_abs_occupancy, std::abs(voxel.occupancy));
  });

  std::cout << "Layered hashed wavelet integrator experiment\n";
  printVoxel("Layered voxel before integration", before);
  printVoxel("Same layered voxel after integration", after);

  const bool occupancy_changed_somewhere = 1e-5f < max_abs_occupancy;
  const bool map_allocated_integration_nodes = size_before < size_after;
  const bool rgb_preserved = approximatelyEqual(before.data.rgb, after.data.rgb);
  const bool traversability_preserved =
      approximatelyEqual(before.data.traversability,
                         after.data.traversability);

  std::cout << "Map size before integration: " << size_before << "\n";
  std::cout << "Map size after integration: " << size_after << "\n";
  std::cout << "Max abs occupancy after integration: "
            << max_abs_occupancy << "\n";
  std::cout << "Occupancy changed through templated hashed wavelet integrator: "
            << (occupancy_changed_somewhere ? "yes" : "no") << "\n";
  std::cout << "Integrator allocated/updated nodes: "
            << (map_allocated_integration_nodes ? "yes" : "no") << "\n";
  std::cout << "RGB preserved by occupancy-only update: "
            << (rgb_preserved ? "yes" : "no") << "\n";
  std::cout << "Traversability preserved by occupancy-only update: "
            << (traversability_preserved ? "yes" : "no") << "\n";

  if (!occupancy_changed_somewhere || !rgb_preserved ||
      !traversability_preserved) {
    return 1;
  }
  return 0;
}
