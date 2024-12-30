#include <strstream>

#include <wavemap/io/stream_conversions.h>

using namespace wavemap;  // NOLINT
int main(int, char**) {
  // Create an empty map for illustration purposes
  HashedWaveletOctreeConfig config;
  HashedWaveletOctree map(config);

  // Create an output stream for illustration purposes
  std::ostrstream output_stream;

  // Save the map
  bool success = io::mapToStream(map, output_stream);
  output_stream.flush();
  success &= output_stream.good();
  CHECK(success);
}
