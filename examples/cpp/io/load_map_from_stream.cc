#include <strstream>

#include <wavemap/io/stream_conversions.h>

int main(int, char**) {
  // Create a smart pointer that will own the loaded map
  wavemap::MapBase::Ptr loaded_map;

  // Create an input stream for illustration purposes
  std::istrstream input_stream{""};

  // Load the map
  const bool success = wavemap::io::streamToMap(input_stream, loaded_map);
}
