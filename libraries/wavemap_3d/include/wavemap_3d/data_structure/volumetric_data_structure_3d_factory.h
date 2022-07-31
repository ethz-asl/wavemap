#ifndef WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_FACTORY_H_
#define WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_FACTORY_H_

#include <string>

#include <wavemap_common/utils/type_utils.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"

namespace wavemap {
enum class VolumetricDataStructure3DType : int {
  kHashedBlocks,
  kOctree,
  kDifferencingOctree,
  kWaveletOctree
};
constexpr std::array kVolumetricDataStructure3DTypeStrs = {
    "hashed_blocks", "octree", "differencing_octree", "wavelet_octree"};
std::string getVolumetricDataStructure3DTypeStr(
    VolumetricDataStructure3DType intersection_type) {
  return kVolumetricDataStructure3DTypeStrs[to_underlying(intersection_type)];
}

class VolumetricDataStructure3DFactory {
 public:
  static VolumetricDataStructure3D::Ptr create(
      const std::string& data_structure_type_name, FloatingPoint min_cell_width,
      std::optional<VolumetricDataStructure3DType> default_data_structure_type =
          std::nullopt);

  static VolumetricDataStructure3D::Ptr create(
      VolumetricDataStructure3DType data_structure_type,
      FloatingPoint min_cell_width);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_FACTORY_H_
