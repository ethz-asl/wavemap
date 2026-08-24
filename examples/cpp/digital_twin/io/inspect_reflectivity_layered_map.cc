#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>

#include <wavemap/layered/local_elevation_classifier.h>

#include "../ouster_mapping/reflectivity_map_layers.h"

namespace reflectivity_map = wavemap::examples::reflectivity_map;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <reflectivity_map.lwvmp>\n";
    return 1;
  }

  reflectivity_map::Definition::Map map(reflectivity_map::makeMapConfig());
  std::string error;
  if (!reflectivity_map::Definition::MapIo::load(
          std::filesystem::path(argv[1]), map, &error)) {
    std::cerr << "Could not load reflectivity layered map: " << error << "\n";
    return 1;
  }

  size_t leaves = 0u;
  size_t observed_occupancy = 0u;
  size_t observed_reflectivity = 0u;
  float min_reflectivity = std::numeric_limits<float>::max();
  float max_reflectivity = std::numeric_limits<float>::lowest();
  size_t out_of_bounds_reflectivity = 0u;
  size_t ground_classes = 0u;
  size_t obstacle_classes = 0u;
  const auto& storage_bounds =
      map.continuousMap()
          .getThresholdConfig()
          .data.get<reflectivity_map::ReflectivityLayer>();
  map.continuousMap().forEachVoxelLeaf(
      [&](const wavemap::OctreeIndex&,
          const reflectivity_map::Definition::Voxel& voxel) {
        ++leaves;
        if (1e-5f < std::abs(voxel.occupancy)) {
          ++observed_occupancy;
        }
        const float reflectivity =
            voxel.data.get<reflectivity_map::ReflectivityLayer>();
        if (1e-6f < std::abs(reflectivity)) {
          ++observed_reflectivity;
        }
        out_of_bounds_reflectivity +=
            reflectivity < storage_bounds.min ||
            storage_bounds.max < reflectivity;
        min_reflectivity = std::min(min_reflectivity, reflectivity);
        max_reflectivity = std::max(max_reflectivity, reflectivity);
      });

  const auto& class_layer =
      map.discreteLayers().get<reflectivity_map::ClassLayer>();
  for (const auto& [parent_key, cell] : class_layer.cells()) {
    (void)parent_key;
    for (const int offset : cell.observed_offsets) {
      const auto exception = cell.exceptions.find(offset);
      const int value = exception == cell.exceptions.end()
                            ? cell.dominant_value
                            : exception->second;
      ground_classes += value == static_cast<int>(
                                     wavemap::layered::GeometricClass::kGround);
      obstacle_classes +=
          value == static_cast<int>(
                       wavemap::layered::GeometricClass::kObstacle);
    }
  }

  std::cout << "Reflectivity layered map inspection\n"
            << "  allocated blocks: "
            << map.continuousMap().getHashMap().size() << "\n"
            << "  leaf nodes: " << leaves << "\n"
            << "  leaves with occupancy evidence: " << observed_occupancy
            << "\n"
            << "  leaves with reflectivity: " << observed_reflectivity << "\n"
            << "  class values: " << class_layer.observedValueCount() << "\n"
            << "    ground: " << ground_classes << "\n"
            << "    obstacle: " << obstacle_classes << "\n";
  std::cout << "  configured reflectivity storage range: ["
            << storage_bounds.min << ", " << storage_bounds.max << "]\n";
  if (leaves) {
    std::cout << "  reconstructed reflectivity range: [" << min_reflectivity
              << ", " << max_reflectivity << "]\n"
              << "  reflectivity values outside storage bounds: "
              << out_of_bounds_reflectivity << "\n";
  }

  return observed_occupancy && observed_reflectivity &&
                 class_layer.observedValueCount()
             ? 0
             : 2;
}
