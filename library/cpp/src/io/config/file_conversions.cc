#include "wavemap/io/config/file_conversions.h"

#include <fstream>

#include "wavemap/io/config/stream_conversions.h"

namespace wavemap::io {
bool fileToParams(const std::filesystem::path& file_path,
                  param::Value& params) {
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
  if (!yamlStreamToParams(file_istream, params)) {
    LOG(WARNING) << "Failed to parse map from file " << file_path << ".";
    return false;
  }

  return true;
}
}  // namespace wavemap::io
