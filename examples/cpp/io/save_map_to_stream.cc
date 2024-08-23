#include <strstream>

#include <wavemap/io/stream_conversions.h>

int main(int, char**) {
  // Create an empty map for illustration purposes
  wavemap::HashedWaveletOctreeConfig config;
  wavemap::HashedWaveletOctree map(config);

  // Create an output stream for illustration purposes
  std::ostrstream output_stream;

  // Save the map
  bool success = wavemap::io::mapToStream(map, output_stream);
  output_stream.flush();
  success &= output_stream.good();
}
