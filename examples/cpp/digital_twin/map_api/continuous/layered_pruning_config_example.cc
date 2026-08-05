#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "../../common/layered_voxel_config.h"

namespace {
struct ExampleRun {
  std::string name;
  LayeredPruningConfig pruning_config;
};

void fillMapWithLayerBoundary(ContinuousWaveletMap& map) {
  for (int x = 0; x < 32; ++x) {
    for (int y = 0; y < 32; ++y) {
      for (int z = 0; z < 32; ++z) {
        const wavemap::FloatingPoint occupancy = 0.5f;

        Rgb color = rgb(0.2f, 0.2f, 0.2f);
        if (16 <= x) {
          color = rgb(0.9f, 0.1f, 0.1f);
        }
        if (16 <= y) {
          color = rgb(0.1f, 0.1f, 0.9f);
        }

        const float traversability = (8 <= z && z < 24) ? 0.85f : 0.25f;
        map.setVoxelValue(wavemap::Index3D{x, y, z},
                          makeLayeredVoxel(occupancy, color, traversability));
      }
    }
  }
}

void runExample(const wavemap::HashedWaveletOctreeConfig& map_config,
                const ExampleRun& run) {
  ContinuousWaveletMap map(map_config, ContinuousWaveletMap::ThresholdConfig{},
                 run.pruning_config);
  fillMapWithLayerBoundary(map);

  const size_t nodes_before = map.size();
  const size_t memory_before = map.getMemoryUsage();

  map.threshold();
  map.prune();

  std::cout << run.name << "\n";
  std::cout << "  nodes before prune: " << nodes_before << "\n";
  std::cout << "  nodes after prune: " << map.size() << "\n";
  std::cout << "  memory before prune bytes: " << memory_before << "\n";
  std::cout << "  memory after prune bytes: " << map.getMemoryUsage() << "\n";
  printVoxel("  sample voxel", map.getVoxelValue(wavemap::Index3D{20, 20, 12}));
  std::cout << "\n";
}
}  // namespace

int main() {
  wavemap::HashedWaveletOctreeConfig map_config;
  map_config.min_cell_width = 0.1f;
  map_config.min_log_odds = -100.f;
  map_config.max_log_odds = 100.f;
  map_config.tree_height = 4;

  // Pruning config quick guide:
  //   scale: detail magnitude that counts as one unit of relevance.
  //   weight: relative importance of this field in the combined score.
  //   combined_threshold: prune if the total score is not above this value.
  // Formula:
  //   score = sum(weight * abs(wavelet_detail) / scale)
  // A field with weight 0 is ignored by pruning.
  const std::vector<ExampleRun> runs{
      {"Occupancy-only pruning",
       makeOccupancyOnlyPruningConfig(/*scale=*/1.f)},
      {"Equal occupancy/RGB/traversability pruning",
       makeEqualLayerPruningConfig(/*scale=*/0.1f)},
      {"Custom pruning: occupancy-heavy, layers still count",
       makeLayeredPruningConfigWithScales(
           /*occupancy_weight=*/4.f, /*rgb_weight=*/1.f,
           /*traversability_weight=*/1.f, /*occupancy_scale=*/1.f,
           /*rgb_scale=*/1.f, /*traversability_scale=*/1.f)}};

  for (const ExampleRun& run : runs) {
    runExample(map_config, run);
  }

  return 0;
}
