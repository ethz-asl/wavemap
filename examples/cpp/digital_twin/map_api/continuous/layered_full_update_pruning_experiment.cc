#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/core/integrator/measurement_model/continuous_beam.h>
#include <wavemap/core/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h>
#include <wavemap/core/integrator/projection_model/spherical_projector.h>

#include "../../common/layered_voxel_config.h"
#include "../../common/layered_voxel_updater.h"

namespace {
struct RunConfig {
  std::string name;
  LayeredPruningConfig pruning_config;
};

struct RunResult {
  std::string name;
  size_t nodes_before_prune = 0u;
  size_t nodes_after_prune = 0u;
  size_t memory_before_prune = 0u;
  size_t memory_after_prune = 0u;
  float mean_occupancy_delta_after_prune = 0.f;
  float mean_rgb_delta_after_prune = 0.f;
  float mean_traversability_delta_after_prune = 0.f;
  float max_occupancy_delta_after_prune = 0.f;
  float max_rgb_delta_after_prune = 0.f;
  float max_traversability_delta_after_prune = 0.f;
};

ContinuousLayers makeLayerObservation(int x, int y, int z) {
  Rgb color = rgb(0.2f, 0.2f, 0.2f);
  if (13 <= x) {
    color = rgb(0.9f, 0.1f, 0.1f);
  }
  if (21 <= y) {
    color = rgb(0.1f, 0.1f, 0.9f);
  }

  const float traversability = (7 <= z && z < 23) ? 0.85f : 0.25f;
  return ContinuousLayers{color, traversability};
}

std::vector<ContinuousLayerObservation> makeLayerObservations() {
  constexpr int kSize = 32;

  std::vector<ContinuousLayerObservation> observations;
  observations.reserve(kSize * kSize * kSize);
  for (int x = 0; x < kSize; ++x) {
    for (int y = 0; y < kSize; ++y) {
      for (int z = 0; z < kSize; ++z) {
        observations.push_back(
            {wavemap::Index3D(x, y, z), makeLayerObservation(x, y, z)});
      }
    }
  }
  return observations;
}

std::shared_ptr<ContinuousWaveletMap> makeMapAfterRayAndLayerUpdates(
    const wavemap::HashedWaveletOctreeConfig& map_config,
    const LayeredPruningConfig& pruning_config,
    const std::vector<ContinuousLayerObservation>& observations) {
  auto map = std::make_shared<ContinuousWaveletMap>(
      map_config, ContinuousWaveletMap::ThresholdConfig{}, pruning_config);

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
  integrator_config.max_range = 4.f;
  integrator_config.max_update_resolution = 0.f;
  integrator_config.termination_update_error = 0.05f;

  wavemap::HashedWaveletIntegratorT<LayeredVoxel> integrator(
      integrator_config, projection_model, posed_range_image,
      beam_offset_image, measurement_model, map);

  wavemap::Pointcloud<> pointcloud;
  pointcloud.resize(3u);
  pointcloud[0] = wavemap::Point3D(1.0f, 0.0f, 0.0f);
  pointcloud[1] = wavemap::Point3D(1.5f, 0.2f, 0.0f);
  pointcloud[2] = wavemap::Point3D(2.0f, -0.2f, 0.1f);
  const wavemap::Transformation3D T_W_C;
  const wavemap::PosedPointcloud<> posed_pointcloud(T_W_C, pointcloud);

  integrator.integrate(posed_pointcloud);
  map->threshold();

  setContinuousLayers(*map, observations);
  map->threshold();
  return map;
}

std::vector<LayeredVoxel> readVoxels(
    const ContinuousWaveletMap& map,
    const std::vector<ContinuousLayerObservation>& observations) {
  std::vector<LayeredVoxel> values;
  values.reserve(observations.size());
  for (const ContinuousLayerObservation& observation : observations) {
    values.push_back(map.getVoxelValue(observation.index));
  }
  return values;
}

RunResult runFullPipelinePruning(
    const wavemap::HashedWaveletOctreeConfig& map_config,
    const RunConfig& run_config,
    const std::vector<ContinuousLayerObservation>& observations) {
  auto map = makeMapAfterRayAndLayerUpdates(
      map_config, run_config.pruning_config, observations);

  RunResult result;
  result.name = run_config.name;
  result.nodes_before_prune = map->size();
  result.memory_before_prune = map->getMemoryUsage();
  const std::vector<LayeredVoxel> before_prune_values =
      readVoxels(*map, observations);

  map->prune();

  result.nodes_after_prune = map->size();
  result.memory_after_prune = map->getMemoryUsage();

  for (size_t i = 0; i < observations.size(); ++i) {
    const LayeredVoxel after = map->getVoxelValue(observations[i].index);
    const LayeredVoxel& before = before_prune_values[i];

    const float occupancy_delta = std::abs(after.occupancy - before.occupancy);
    const float rgb_delta = sumAbsDiff(after.data.rgb, before.data.rgb);
    const float traversability_delta =
        std::abs(after.data.traversability - before.data.traversability);

    result.mean_occupancy_delta_after_prune += occupancy_delta;
    result.mean_rgb_delta_after_prune += rgb_delta;
    result.mean_traversability_delta_after_prune += traversability_delta;
    result.max_occupancy_delta_after_prune =
        std::max(result.max_occupancy_delta_after_prune, occupancy_delta);
    result.max_rgb_delta_after_prune =
        std::max(result.max_rgb_delta_after_prune, rgb_delta);
    result.max_traversability_delta_after_prune =
        std::max(result.max_traversability_delta_after_prune,
                 traversability_delta);
  }

  const float sample_count = static_cast<float>(observations.size());
  result.mean_occupancy_delta_after_prune /= sample_count;
  result.mean_rgb_delta_after_prune /= sample_count;
  result.mean_traversability_delta_after_prune /= sample_count;
  return result;
}

void printRunResult(const RunResult& result) {
  std::cout << result.name << "\n";
  std::cout << "  nodes before prune: " << result.nodes_before_prune << "\n";
  std::cout << "  nodes after prune: " << result.nodes_after_prune << "\n";
  std::cout << "  memory before prune bytes: " << result.memory_before_prune
            << "\n";
  std::cout << "  memory after prune bytes: " << result.memory_after_prune
            << "\n";
  std::cout << "  mean occupancy delta after prune: "
            << result.mean_occupancy_delta_after_prune << "\n";
  std::cout << "  max occupancy delta after prune: "
            << result.max_occupancy_delta_after_prune << "\n";
  std::cout << "  mean rgb delta after prune: "
            << result.mean_rgb_delta_after_prune << "\n";
  std::cout << "  max rgb delta after prune: "
            << result.max_rgb_delta_after_prune << "\n";
  std::cout << "  mean traversability delta after prune: "
            << result.mean_traversability_delta_after_prune << "\n";
  std::cout << "  max traversability delta after prune: "
            << result.max_traversability_delta_after_prune << "\n";
}
}  // namespace

int main() {
  wavemap::HashedWaveletOctreeConfig map_config;
  map_config.min_cell_width = 0.1f;
  map_config.min_log_odds = -2.f;
  map_config.max_log_odds = 4.f;
  map_config.tree_height = 4;

  const std::vector<ContinuousLayerObservation> observations =
      makeLayerObservations();

  const std::vector<RunConfig> run_configs{
      {"Full pipeline pruning: occupancy-only",
       makeLayeredPruningConfigWithScales(1.f, 0.f, 0.f, 1.f, 1.f, 1.f)},
      {"Full pipeline pruning: equal layers",
       makeLayeredPruningConfigWithScales(1.f, 1.f, 1.f, 1.f, 0.1f, 0.1f)},
      {"Full pipeline pruning: occupancy-heavy",
       makeLayeredPruningConfigWithScales(4.f, 1.f, 1.f, 1.f, 0.75f, 0.75f)}};

  std::cout << "Layered full update + pruning experiment\n";
  std::cout << "Occupancy is integrated with the original ray-based Wavemap "
               "pipeline. Continuous layers are then set from per-voxel "
               "observations and the map is pruned with different weighted "
               "policies.\n";
  std::cout << "Per-voxel layer observations: " << observations.size()
            << "\n\n";

  bool saw_compression = false;
  bool saw_layer_preservation = false;
  for (const RunConfig& run_config : run_configs) {
    const RunResult result =
        runFullPipelinePruning(map_config, run_config, observations);
    printRunResult(result);
    std::cout << "\n";

    saw_compression |= result.nodes_after_prune < result.nodes_before_prune;
    saw_layer_preservation |= result.mean_rgb_delta_after_prune < 1e-4f &&
                              result.mean_traversability_delta_after_prune <
                                  1e-4f;
  }

  if (!saw_compression || !saw_layer_preservation) {
    return 1;
  }
  return 0;
}
