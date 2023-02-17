#ifndef WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_FACTORY_H_
#define WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_FACTORY_H_

#include <string>

#include <wavemap/utils/config_utils.h>
#include <wavemap/utils/factory_utils.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"

namespace wavemap {
struct VolumetricDataStructure3DType
    : TypeSelector<VolumetricDataStructure3DType> {
  using TypeSelector<VolumetricDataStructure3DType>::TypeSelector;

  enum Id : TypeId {
    kHashedBlocks,
    kOctree,
    kDifferencingOctree,
    kWaveletOctree
  };

  static constexpr std::array names = {"hashed_blocks", "octree",
                                       "differencing_octree", "wavelet_octree"};
};

class VolumetricDataStructure3DFactory {
 public:
  static VolumetricDataStructure3D::Ptr create(
      const param::Map& params,
      std::optional<VolumetricDataStructure3DType> default_data_structure_type =
          std::nullopt);

  static VolumetricDataStructure3D::Ptr create(
      VolumetricDataStructure3DType data_structure_type,
      const param::Map& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_FACTORY_H_
