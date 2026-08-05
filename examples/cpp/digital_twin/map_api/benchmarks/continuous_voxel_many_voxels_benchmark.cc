#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "../../common/layered_voxel_config.h"

LayeredVoxel makeVoxel(int x, int y, int z) {
  const auto occupancy =
      static_cast<wavemap::FloatingPoint>(1 + ((3 * x + 5 * y + 7 * z) % 11));
  const float traversability = static_cast<float>(x + y + z) / 18.f;
  const float r = static_cast<float>(x) / 7.f;
  const float g = static_cast<float>(y) / 7.f;
  const float b = static_cast<float>(z) / 3.f;
  return makeLayeredVoxel(occupancy, rgb(r, g, b), traversability);
}

float absError(const LayeredVoxel& read, const LayeredVoxel& expected) {
  return std::abs(read.occupancy - expected.occupancy) +
         std::abs(read.data.traversability - expected.data.traversability) +
         std::abs(read.data.rgb.r - expected.data.rgb.r) +
         std::abs(read.data.rgb.g - expected.data.rgb.g) +
         std::abs(read.data.rgb.b - expected.data.rgb.b);
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 4;

  std::vector<std::pair<wavemap::Index3D, LayeredVoxel>> samples;
  samples.reserve(8 * 8 * 4);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 4; ++z) {
        samples.emplace_back(wavemap::Index3D(x, y, z), makeVoxel(x, y, z));
      }
    }
  }

  ContinuousWaveletMap map(config);

  const auto write_start = std::chrono::steady_clock::now();
  for (const auto& [index, voxel] : samples) {
    map.setVoxelValue(index, voxel);
  }
  const auto write_end = std::chrono::steady_clock::now();

  float abs_error_sum = 0.f;
  float max_abs_error = 0.f;

  const auto read_start = std::chrono::steady_clock::now();
  for (const auto& [index, expected] : samples) {
    const LayeredVoxel read = map.getVoxelValue(index);
    const float error = absError(read, expected);
    abs_error_sum += error;
    max_abs_error = std::max(max_abs_error, error);
  }
  const auto read_end = std::chrono::steady_clock::now();

  const auto write_us =
      std::chrono::duration_cast<std::chrono::microseconds>(write_end -
                                                            write_start)
          .count();
  const auto read_us =
      std::chrono::duration_cast<std::chrono::microseconds>(read_end -
                                                            read_start)
          .count();

  size_t non_zero_leaves = 0u;
  map.forEachVoxelLeaf(
      [&non_zero_leaves](const wavemap::OctreeIndex& /*index*/,
                         const LayeredVoxel& voxel) {
        const bool non_zero = std::abs(voxel.occupancy) > 1e-5f ||
                              std::abs(voxel.data.traversability) > 1e-5f ||
                              std::abs(voxel.data.rgb.r) > 1e-5f ||
                              std::abs(voxel.data.rgb.g) > 1e-5f ||
                              std::abs(voxel.data.rgb.b) > 1e-5f;
        if (non_zero) {
          ++non_zero_leaves;
        }
      });

  std::cout << "samples: " << samples.size() << "\n";
  std::cout << "combined abs error sum: " << abs_error_sum << "\n";
  std::cout << "max combined abs error: " << max_abs_error << "\n";
  std::cout << "non-zero leaves: " << non_zero_leaves << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";
  std::cout << "memory usage bytes: " << map.getMemoryUsage() << "\n";
  std::cout << "write time [us]: " << write_us << "\n";
  std::cout << "read time [us]: " << read_us << "\n";

  return 0;
}
