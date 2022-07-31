#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_FACTORY_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_FACTORY_H_

#include <string>

#include <wavemap_common/utils/type_utils.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
enum class VolumetricDataStructure2DType : int {
  kDenseGrid,
  kHashedBlocks,
  kQuadtree,
  kDifferencingQuadtree,
  kWaveletQuadtree
};
constexpr std::array<const char*, 5> kVolumetricDataStructure2DTypeStrs = {
    "dense_grid", "hashed_blocks", "quadtree", "differencing_quadtree",
    "wavelet_quadtree"};
std::string getVolumetricDataStructure2DTypeStr(
    VolumetricDataStructure2DType intersection_type) {
  return kVolumetricDataStructure2DTypeStrs[to_underlying(intersection_type)];
}

class VolumetricDataStructure2DFactory {
 public:
  static VolumetricDataStructure2D::Ptr create(
      const std::string& data_structure_type_name, FloatingPoint min_cell_width,
      std::optional<VolumetricDataStructure2DType> default_data_structure_type =
          std::nullopt);

  static VolumetricDataStructure2D::Ptr create(
      VolumetricDataStructure2DType data_structure_type,
      FloatingPoint min_cell_width);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_FACTORY_H_
