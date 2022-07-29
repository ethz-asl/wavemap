#include "wavemap_2d/data_structure/volumetric_data_structure_2d_factory.h"

#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>

#include "wavemap_2d/data_structure/dense_grid.h"
#include "wavemap_2d/data_structure/hashed_blocks_2d.h"
#include "wavemap_2d/data_structure/volumetric_differencing_quadtree.h"
#include "wavemap_2d/data_structure/volumetric_quadtree.h"
#include "wavemap_2d/data_structure/wavelet_tree_2d.h"

namespace wavemap {
VolumetricDataStructure2D::Ptr VolumetricDataStructure2DFactory::create(
    const std::string& data_structure_type_name, FloatingPoint min_cell_width,
    std::optional<VolumetricDataStructure2DType> default_data_structure_type) {
  for (size_t type_idx = 0;
       type_idx < kVolumetricDataStructure2DTypeStrs.size(); ++type_idx) {
    if (data_structure_type_name ==
        kVolumetricDataStructure2DTypeStrs[type_idx]) {
      return create(static_cast<VolumetricDataStructure2DType>(type_idx),
                    min_cell_width);
    }
  }

  if (default_data_structure_type.has_value()) {
    LOG(WARNING) << "Requested creation of unknown data structure type \""
                 << data_structure_type_name << "\". Default type \""
                 << getVolumetricDataStructure2DTypeStr(
                        default_data_structure_type.value())
                 << "\" will be created instead.";
    return create(default_data_structure_type.value(), min_cell_width);
  } else {
    LOG(ERROR) << "Requested creation of unknown data structure type \""
               << data_structure_type_name
               << "\" and no default was set. Returning nullptr.";
  }
  return nullptr;
}

VolumetricDataStructure2D::Ptr VolumetricDataStructure2DFactory::create(
    VolumetricDataStructure2DType data_structure_type,
    FloatingPoint min_cell_width) {
  switch (data_structure_type) {
    case VolumetricDataStructure2DType::kDenseGrid:
      return std::make_shared<DenseGrid<SaturatingOccupancyCell>>(
          min_cell_width);
    case VolumetricDataStructure2DType::kHashedBlocks:
      return std::make_shared<HashedBlocks2D<SaturatingOccupancyCell>>(
          min_cell_width);
    case VolumetricDataStructure2DType::kQuadtree:
      return std::make_shared<VolumetricQuadtree<SaturatingOccupancyCell>>(
          min_cell_width);
    case VolumetricDataStructure2DType::kDifferencingQuadtree:
      return std::make_shared<
          VolumetricDifferencingQuadtree<SaturatingOccupancyCell>>(
          min_cell_width);
    case VolumetricDataStructure2DType::kWaveletTree:
      return std::make_shared<WaveletTree2D<SaturatingOccupancyCell>>(
          min_cell_width);
    default:
      LOG(ERROR) << "Attempted to create unknown data structure type: "
                 << to_underlying(data_structure_type)
                 << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
