#include <wavemap/io/file_conversions.h>

int main(int, char**) {
  // Create an empty map for illustration purposes
  wavemap::HashedWaveletOctreeConfig config;
  wavemap::HashedWaveletOctree map(config);

  // Save the map
  const bool success = wavemap::io::mapToFile(map, "/path/to/your/map.wvmp");
}
