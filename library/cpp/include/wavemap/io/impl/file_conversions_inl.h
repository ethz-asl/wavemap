#ifndef WAVEMAP_IO_IMPL_FILE_CONVERSIONS_INL_H_
#define WAVEMAP_IO_IMPL_FILE_CONVERSIONS_INL_H_

#include <cerrno>
#include <cstring>
#include <fstream>

namespace wavemap::io {
template <typename CellDataT, typename CellDataSerializerT>
bool mapToFile(const HashedWaveletOctreeT<CellDataT>& map, const std::filesystem::path& file_path) {
  if (file_path.empty()) {
    LOG(WARNING) << "Could open file for writing. Specified file path is empty.";
    return false;
  }

  std::ofstream file_ostream(file_path,
                             std::ofstream::out | std::ofstream::binary);
  if (!file_ostream.is_open()) {
    LOG(WARNING) << "Could not open file " << file_path
                 << " for writing. Error: " << strerror(errno);
    return false;
  }

  if (!mapToStream<CellDataT, CellDataSerializerT>(map, file_ostream)) {
    return false;
  }

  file_ostream.close();
  return static_cast<bool>(file_ostream);
}

template <typename CellDataT, typename CellDataSerializerT>
bool fileToMap(const std::filesystem::path& file_path, typename HashedWaveletOctreeT<CellDataT>::Ptr& map) {
  if (file_path.empty()) {
    LOG(WARNING) << "Could not open file for reading. Specified file path is empty.";
    return false;
  }

  std::ifstream file_istream(file_path,
                             std::ifstream::in | std::ifstream::binary);
  if (!file_istream.is_open()) {
    LOG(WARNING) << "Could not open file " << file_path << " for reading. Error: " << strerror(errno);
    return false;
  }

  if (!streamToMap<CellDataT, CellDataSerializerT>(file_istream, map)) {
    LOG(WARNING) << "Failed to parse map from file " << file_path << ".";
    return false;
  }

  return true;
}
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_IMPL_FILE_CONVERSIONS_INL_H_
