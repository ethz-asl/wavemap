#include "wavemap_io/file_conversions.h"

#include <fstream>

namespace wavemap::io {
bool mapToFile(const VolumetricDataStructureBase& map,
               const std::string& file_path) {
  CHECK(!file_path.empty());

  // Open the file for writing
  std::ofstream file_ostream(file_path,
                             std::ofstream::out | std::ofstream::binary);
  if (!file_ostream.is_open()) {
    LOG(WARNING) << "Could not open file '" << file_path << "' for writing.";
    return false;
  }

  // Serialize to bytestream
  if (!mapToStream(map, file_ostream)) {
    return false;
  }

  // Close the file and communicate whether writing succeeded
  file_ostream.close();
  return static_cast<bool>(file_ostream);
}

bool fileToMap(const std::string& file_path,
               VolumetricDataStructureBase::Ptr& map) {
  CHECK(!file_path.empty());

  // Open the file for reading
  std::ifstream file_istream(file_path,
                             std::ifstream::in | std::ifstream::binary);
  if (!file_istream.is_open()) {
    LOG(WARNING) << "Could not open file '" << file_path << "' for reading.";
    return false;
  }

  // Deserialize from bytestream
  if (!streamToMap(file_istream, map)) {
    LOG(WARNING) << "Failed to parse map proto from file '" << file_path
                 << "'.";
    return false;
  }

  return true;
}
}  // namespace wavemap::io
