#include <filesystem>
#include <iostream>

#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/io/file_conversions.h>

int main() {
  const std::filesystem::path input_path = "/home/ci/data/maps/wavemap1.wvmp";
  const std::filesystem::path output_path = "/home/ci/data/maps/map_copy.wvmp";

  wavemap::MapBase::Ptr map;

  if (!wavemap::io::fileToMap(input_path, map)) {
    std::cerr << "Failed to load map from: " << input_path << "\n";
    return 1;
  }

  if (!map) {
    std::cerr << "Loaded map pointer is null.\n";
    return 1;
  }
  std::cout << "Loaded map from: " << input_path << "\n";

  map->threshold();

  size_t occupied_like = 0;
  size_t free_like = 0;

  map->forEachLeaf([&](const wavemap::OctreeIndex&, float value) {
    if (value > 0.f) {
      ++occupied_like;
    } else if (value < 0.f) {
      ++free_like;
    }
  });

  std::cout << "Occupied-like leaves: " << occupied_like << "\n";
  std::cout << "Free-like leaves: " << free_like << "\n";

  if (std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map)) {
    std::cout << "Loaded as hashed_wavelet_octree\n";
  }

  if (std::dynamic_pointer_cast<wavemap::HashedChunkedWaveletOctree>(map)) {
    std::cout << "Loaded as hashed_chunked_wavelet_octree\n";
  }

  if (!wavemap::io::mapToFile(*map, output_path)) {
    std::cerr << "Failed to save map to: " << output_path << "\n";
    return 1;
  }

  std::cout << "Saved map to: " << output_path << "\n";

  return 0;
}
