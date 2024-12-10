#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/io/map/file_conversions.h>

int main(int, char**) {
  // Create an empty map for illustration purposes
  wavemap::HashedChunkedWaveletOctreeConfig config;
  wavemap::HashedChunkedWaveletOctree map(config);

  // Save the map
  const bool success = wavemap::io::mapToFile(map, "/path/to/your/map.wvmp");
}
