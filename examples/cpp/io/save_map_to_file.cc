#include <wavemap/io/file_conversions.h>

using namespace wavemap;  // NOLINT
int main(int, char**) {
  // Create an empty map for illustration purposes
  HashedWaveletOctreeConfig config;
  HashedWaveletOctree map(config);

  // Save the map
  const bool success = io::mapToFile(map, "/path/to/your/map.wvmp");
  CHECK(success);
}
