#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

wavemap::FloatingPoint makeOccupancy(int x, int y, int z) {
  return static_cast<wavemap::FloatingPoint>(1 + ((3 * x + 5 * y + 7 * z) % 11));
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 4;

  std::vector<std::pair<wavemap::Index3D, wavemap::FloatingPoint>> samples;
  samples.reserve(8 * 8 * 4);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 4; ++z) {
        samples.emplace_back(wavemap::Index3D(x, y, z), makeOccupancy(x, y, z));
      }
    }
  }

  wavemap::HashedWaveletOctree map(config);

  const auto write_start = std::chrono::steady_clock::now();
  for (const auto& [index, occupancy] : samples) {
    map.setCellValue(index, occupancy);
  }
  const auto write_end = std::chrono::steady_clock::now();

  wavemap::FloatingPoint abs_error_sum = 0.f;
  wavemap::FloatingPoint max_abs_error = 0.f;

  const auto read_start = std::chrono::steady_clock::now();
  for (const auto& [index, expected] : samples) {
    const wavemap::FloatingPoint read = map.getCellValue(index);
    const wavemap::FloatingPoint abs_error = std::abs(read - expected);
    abs_error_sum += abs_error;
    max_abs_error = std::max(max_abs_error, abs_error);
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
                         wavemap::FloatingPoint value) {
        if (std::abs(value) > 1e-5f) {
          ++non_zero_leaves;
        }
      });

  std::cout << "samples: " << samples.size() << "\n";
  std::cout << "abs error sum: " << abs_error_sum << "\n";
  std::cout << "max abs error: " << max_abs_error << "\n";
  std::cout << "non-zero leaves: " << non_zero_leaves << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";
  std::cout << "memory usage bytes: " << map.getMemoryUsage() << "\n";
  std::cout << "write time [us]: " << write_us << "\n";
  std::cout << "read time [us]: " << read_us << "\n";

  return 0;
}
