#include <wavemap_io/file_conversions.h>

int main(int, char**) {
  // Create an empty map for illustration purposes
  wavemap::HashedWaveletOctreeConfig config;
  wavemap::HashedWaveletOctree map(config);

  // Save the map
  wavemap::io::mapToFile(map, "/some/path/to/your/map.wvmp");
}
