#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>

#include "../common/field_layered_map_config.h"

namespace field_map = wavemap::examples::field_map;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <field_map.lwvmp>\n";
    return 1;
  }

  field_map::Map map(field_map::makeDefaultConfig());
  std::string error;
  if (!field_map::MapIo::load(std::filesystem::path(argv[1]), map, &error)) {
    std::cerr << "Could not load field layered map: " << error << "\n";
    return 1;
  }

  size_t leaves = 0u;
  size_t observed_occupancy = 0u;
  size_t observed_reflectivity = 0u;
  float min_reflectivity = std::numeric_limits<float>::max();
  float max_reflectivity = std::numeric_limits<float>::lowest();
  map.continuousMap().forEachVoxelLeaf(
      [&](const wavemap::OctreeIndex&, const field_map::Voxel& voxel) {
        ++leaves;
        if (1e-5f < std::abs(voxel.occupancy)) {
          ++observed_occupancy;
        }
        const float reflectivity =
            voxel.data.get<field_map::ReflectivityLayer>();
        if (1e-5f < std::abs(reflectivity)) {
          ++observed_reflectivity;
          min_reflectivity = std::min(min_reflectivity, reflectivity);
          max_reflectivity = std::max(max_reflectivity, reflectivity);
        }
      });

  std::cout << "Field layered map inspection\n"
            << "  allocated blocks: "
            << map.continuousMap().getHashMap().size() << "\n"
            << "  leaf nodes: " << leaves << "\n"
            << "  leaves with occupancy evidence: " << observed_occupancy
            << "\n"
            << "  leaves with reflectivity: " << observed_reflectivity << "\n";
  if (observed_reflectivity) {
    std::cout << "  nonzero reflectivity range: [" << min_reflectivity << ", "
              << max_reflectivity << "]\n";
  }

  return observed_occupancy && observed_reflectivity ? 0 : 2;
}

