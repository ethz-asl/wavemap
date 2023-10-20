#ifndef WAVEMAP_IO_FILE_CONVERSIONS_H_
#define WAVEMAP_IO_FILE_CONVERSIONS_H_

#include <filesystem>

#include <wavemap/map/volumetric_data_structure_base.h>

#include "wavemap_io/stream_conversions.h"

namespace wavemap::io {
bool mapToFile(const VolumetricDataStructureBase& map,
               const std::filesystem::path& file_path);
bool fileToMap(const std::filesystem::path& file_path,
               VolumetricDataStructureBase::Ptr& map);
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_FILE_CONVERSIONS_H_
