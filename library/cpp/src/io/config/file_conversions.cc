#include "wavemap/io/config/file_conversions.h"

#include <fstream>

#include "wavemap/io/config/stream_conversions.h"

namespace wavemap::io {
std::optional<param::Value> yamlFileToParams(
    const std::filesystem::path& file_path) {
  if (file_path.empty()) {
    LOG(WARNING)
        << "Could not open file for reading. Specified file path is empty.";
    return std::nullopt;
  }

  // Open the file for reading
  std::ifstream file_istream(file_path,
                             std::ifstream::in | std::ifstream::binary);
  if (!file_istream.is_open()) {
    LOG(WARNING) << "Could not open file " << file_path
                 << " for reading. Error: " << strerror(errno);
    return std::nullopt;
  }

  // Deserialize from bytestream
  if (auto params = yamlStreamToParams(file_istream); params) {
    return params;
  }
  LOG(WARNING) << "Failed to parse map from file " << file_path << ".";
  return std::nullopt;
}
}  // namespace wavemap::io
