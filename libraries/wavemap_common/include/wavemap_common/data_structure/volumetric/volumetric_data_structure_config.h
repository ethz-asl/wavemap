#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_CONFIG_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_CONFIG_H_

#include "wavemap_common/common.h"
#include "wavemap_common/utils/config_utils.h"

namespace wavemap {
struct VolumetricDataStructureConfig {
  FloatingPoint min_cell_width = 0.1f;

  static VolumetricDataStructureConfig fromParams(const param::Map& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_CONFIG_H_
