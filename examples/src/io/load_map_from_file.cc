#include <wavemap_io/file_conversions.h>

int main(int, char**) {
  // Create a smart pointer that will own the loaded map
  wavemap::VolumetricDataStructureBase::Ptr loaded_map;

  // Load the map
  wavemap::io::fileToMap("/some/path/to/your/map.wvmp", loaded_map);
}
