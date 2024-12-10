#include <wavemap/io/map/file_conversions.h>

int main(int, char**) {
  // Create a smart pointer that will own the loaded map
  wavemap::MapBase::Ptr loaded_map;

  // Load the map
  const bool success =
      wavemap::io::fileToMap("/path/to/your/map.wvmp", loaded_map);
}
