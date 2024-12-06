#include "wavemap/io/map/file_conversions.h"

#include <fstream>

#include "wavemap/io/map/stream_conversions.h"

namespace wavemap::io {
bool mapToFile(const MapBase& map, const std::filesystem::path& file_path) {
  if (file_path.empty()) {
    LOG(WARNING)
        << "Could open file for writing. Specified file path is empty.";
    return false;
  }

  // Open the file for writing
  std::ofstream file_ostream(file_path,
                             std::ofstream::out | std::ofstream::binary);
  if (!file_ostream.is_open()) {
    LOG(WARNING) << "Could not open file " << file_path
                 << " for writing. Error: " << strerror(errno);
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

bool fileToMap(const std::filesystem::path& file_path, MapBase::Ptr& map) {
  if (file_path.empty()) {
    LOG(WARNING)
        << "Could not open file for reading. Specified file path is empty.";
    return false;
  }

  // Open the file for reading
  std::ifstream file_istream(file_path,
                             std::ifstream::in | std::ifstream::binary);
  if (!file_istream.is_open()) {
    LOG(WARNING) << "Could not open file " << file_path
                 << " for reading. Error: " << strerror(errno);
    return false;
  }

  // Deserialize from bytestream
  if (!streamToMap(file_istream, map)) {
    LOG(WARNING) << "Failed to parse map from file " << file_path << ".";
    return false;
  }

  return true;
}
}  // namespace wavemap::io
