#include <wavemap/io/file_conversions.h>

using namespace wavemap;  // NOLINT
int main(int, char**) {
  // Create a smart pointer that will own the loaded map
  MapBase::Ptr loaded_map;

  // Load the map
  const bool success = io::fileToMap("/path/to/your/map.wvmp", loaded_map);
  CHECK(success);
}
