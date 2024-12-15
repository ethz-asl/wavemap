#include <strstream>

#include <wavemap/io/stream_conversions.h>

using namespace wavemap;  // NOLINT
int main(int, char**) {
  // Create a smart pointer that will own the loaded map
  MapBase::Ptr loaded_map;

  // Create an input stream for illustration purposes
  std::istrstream input_stream{""};

  // Load the map
  const bool success = io::streamToMap(input_stream, loaded_map);
  CHECK(success);
}
