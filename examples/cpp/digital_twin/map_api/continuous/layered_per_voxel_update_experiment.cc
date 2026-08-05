#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/core/integrator/measurement_model/continuous_beam.h>
#include <wavemap/core/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h>
#include <wavemap/core/integrator/projection_model/spherical_projector.h>

#include "../../common/layered_voxel_config.h"
#include "../../common/layered_voxel_updater.h"

namespace {
bool approximatelyEqual(float lhs, float rhs, float tolerance = 1e-4f) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool approximatelyEqual(const Rgb& lhs, const Rgb& rhs,
                        float tolerance = 1e-4f) {
  return sumAbsDiff(lhs, rhs) <= tolerance;
}

void printObservationResult(const ContinuousLayerObservation& observation,
                            const LayeredVoxel& before,
                            const LayeredVoxel& after) {
  std::cout << "Voxel " << observation.index.transpose() << "\n";
  std::cout << "  occupancy before layer update: " << before.occupancy << "\n";
  std::cout << "  occupancy after layer update:  " << after.occupancy << "\n";
  std::cout << "  requested layer data: trav="
            << observation.data.traversability << " rgb=("
            << observation.data.rgb.r << ", " << observation.data.rgb.g
            << ", " << observation.data.rgb.b << ")\n";
  printVoxel("  stored voxel", after);
}
}  // namespace

int main() {
  wavemap::HashedWaveletOctreeConfig map_config;
  map_config.min_cell_width = 0.1f;
  map_config.min_log_odds = -2.f;
  map_config.max_log_odds = 4.f;
  map_config.tree_height = 4;

  auto map = std::make_shared<ContinuousWaveletMap>(map_config);

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

  integrator.integrate(posed_pointcloud);
  map->threshold();

  std::vector<wavemap::Index3D> updated_indices;
  map->forEachVoxelLeaf(
      [&updated_indices](const wavemap::OctreeIndex& node_index,
                         const LayeredVoxel& voxel) {
        if (updated_indices.size() < 3u && 1e-5f < std::abs(voxel.occupancy)) {
          updated_indices.push_back(
              wavemap::convert::nodeIndexToMinCornerIndex(node_index));
        }
      });

  if (updated_indices.size() < 3u) {
    std::cout << "Not enough occupancy-updated voxels found.\n";
    return 1;
  }

  const std::vector<ContinuousLayerObservation> observations{
      {updated_indices[0], ContinuousLayers{rgb(0.9f, 0.1f, 0.1f), 0.8f}},
      {updated_indices[1], ContinuousLayers{rgb(0.1f, 0.8f, 0.2f), 0.4f}},
      {updated_indices[2], ContinuousLayers{rgb(0.2f, 0.2f, 0.9f), 0.6f}}};

  std::vector<LayeredVoxel> before_layer_update;
  before_layer_update.reserve(observations.size());
  for (const ContinuousLayerObservation& observation : observations) {
    before_layer_update.push_back(map->getVoxelValue(observation.index));
  }

  setContinuousLayers(*map, observations);
  map->threshold();

  bool all_occupancy_preserved = true;
  bool all_layers_updated = true;

  std::cout << "Layered per-voxel update experiment\n";
  std::cout << "Occupancy was integrated through the original ray-based "
               "Wavemap pipeline. Continuous layers are then set from "
               "per-voxel observations using a replacement policy.\n";

  for (size_t i = 0; i < observations.size(); ++i) {
    const LayeredVoxel after = map->getVoxelValue(observations[i].index);
    printObservationResult(observations[i], before_layer_update[i], after);

    all_occupancy_preserved &= approximatelyEqual(
        before_layer_update[i].occupancy, after.occupancy);
    all_layers_updated &= approximatelyEqual(observations[i].data.rgb,
                                             after.data.rgb, 1e-3f);
    all_layers_updated &= approximatelyEqual(
        observations[i].data.traversability, after.data.traversability, 1e-3f);
  }

  std::cout << "Occupancy preserved by per-voxel layer update: "
            << (all_occupancy_preserved ? "yes" : "no") << "\n";
  std::cout << "Continuous layers updated from per-voxel observations: "
            << (all_layers_updated ? "yes" : "no") << "\n";

  if (!all_occupancy_preserved || !all_layers_updated) {
    return 1;
  }
  return 0;
}
