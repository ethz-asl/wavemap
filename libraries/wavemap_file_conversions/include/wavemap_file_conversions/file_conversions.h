#ifndef WAVEMAP_FILE_CONVERSIONS_FILE_CONVERSIONS_H_
#define WAVEMAP_FILE_CONVERSIONS_FILE_CONVERSIONS_H_

#include <string>

#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

#include "wavemap_file_conversions/proto_conversions.h"

namespace wavemap::convert {
bool mapToFile(const VolumetricDataStructureBase& map,
               const std::string& file_path);
bool fileToMap(const std::string& file_path,
               VolumetricDataStructureBase::Ptr& map);
}  // namespace wavemap::convert

#endif  // WAVEMAP_FILE_CONVERSIONS_FILE_CONVERSIONS_H_
