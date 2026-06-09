#ifndef WAVEMAP_IO_FILE_CONVERSIONS_H_
#define WAVEMAP_IO_FILE_CONVERSIONS_H_

#include <filesystem>

#include "wavemap/core/map/map_base.h"
#include "wavemap/io/stream_conversions.h"

namespace wavemap::io {
bool mapToFile(const MapBase& map, const std::filesystem::path& file_path);
bool fileToMap(const std::filesystem::path& file_path, MapBase::Ptr& map);
template <typename CellDataT, typename CellDataSerializerT>
bool mapToFile(const HashedWaveletOctreeT<CellDataT>& map, const std::filesystem::path& file_path);

template <typename CellDataT, typename CellDataSerializerT>
bool fileToMap(const std::filesystem::path& file_path, typename HashedWaveletOctreeT<CellDataT>::Ptr& map);
}  // namespace wavemap::io

#include "wavemap/io/impl/file_conversions_inl.h"

#endif  // WAVEMAP_IO_FILE_CONVERSIONS_H_
