#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_FACTORY_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_FACTORY_H_

#include <string>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/utils/config_utils.h"
#include "wavemap/utils/factory_utils.h"

namespace wavemap {
struct VolumetricDataStructureType : TypeSelector<VolumetricDataStructureType> {
  using TypeSelector<VolumetricDataStructureType>::TypeSelector;

  enum Id : TypeId { kHashedBlocks, kOctree, kWaveletOctree };

  static constexpr std::array names = {"hashed_blocks", "octree",
                                       "wavelet_octree"};
};

class VolumetricDataStructureFactory {
 public:
  static VolumetricDataStructureBase::Ptr create(
      const param::Map& params,
      std::optional<VolumetricDataStructureType> default_data_structure_type =
          std::nullopt);

  static VolumetricDataStructureBase::Ptr create(
      VolumetricDataStructureType data_structure_type,
      const param::Map& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_FACTORY_H_
