#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_FACTORY_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_FACTORY_H_

#include <string>

#include <wavemap_common/utils/config_utils.h>
#include <wavemap_common/utils/factory_utils.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
struct VolumetricDataStructure2DType
    : TypeSelector<VolumetricDataStructure2DType> {
  using TypeSelector<VolumetricDataStructure2DType>::TypeSelector;

  enum Id : TypeId {
    kDenseGrid,
    kHashedBlocks,
    kQuadtree,
    kDifferencingQuadtree,
    kWaveletQuadtree
  };

  static constexpr std::array names = {"dense_grid", "hashed_blocks",
                                       "quadtree", "differencing_quadtree",
                                       "wavelet_quadtree"};
};

class VolumetricDataStructure2DFactory {
 public:
  static VolumetricDataStructure2D::Ptr create(
      const param::Map& params,
      std::optional<VolumetricDataStructure2DType> default_data_structure_type =
          std::nullopt);

  static VolumetricDataStructure2D::Ptr create(
      VolumetricDataStructure2DType data_structure_type,
      const param::Map& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_FACTORY_H_
