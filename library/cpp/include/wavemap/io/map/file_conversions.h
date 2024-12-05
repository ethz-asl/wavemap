#ifndef WAVEMAP_IO_MAP_FILE_CONVERSIONS_H_
#define WAVEMAP_IO_MAP_FILE_CONVERSIONS_H_

#include <filesystem>

#include "wavemap/core/map/map_base.h"

namespace wavemap::io {
bool mapToFile(const MapBase& map, const std::filesystem::path& file_path);
bool fileToMap(const std::filesystem::path& file_path, MapBase::Ptr& map);
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_MAP_FILE_CONVERSIONS_H_
