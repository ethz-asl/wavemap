#include "wavemap_2d/data_structure/volumetric_data_structure_2d_factory.h"

#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>

#include "wavemap_2d/data_structure/dense_grid.h"
#include "wavemap_2d/data_structure/hashed_blocks_2d.h"
#include "wavemap_2d/data_structure/volumetric_differencing_quadtree.h"
#include "wavemap_2d/data_structure/volumetric_quadtree.h"
#include "wavemap_2d/data_structure/wavelet_quadtree.h"

namespace wavemap {
VolumetricDataStructure2D::Ptr VolumetricDataStructure2DFactory::create(
    const param::Map& params,
    std::optional<VolumetricDataStructure2DType> default_data_structure_type) {
  std::string error_msg;
  auto type = VolumetricDataStructure2DType::fromParamMap(params, error_msg);
  if (type.isValid()) {
    return create(type, params);
  }

  if (default_data_structure_type.has_value()) {
    type = default_data_structure_type.value();
    LOG(WARNING) << error_msg << " Default type \"" << type.toStr()
                 << "\" will be created instead.";
    return create(default_data_structure_type.value(), params);
  }

  LOG(ERROR) << error_msg << "No default was set. Returning nullptr.";
  return nullptr;
}

VolumetricDataStructure2D::Ptr VolumetricDataStructure2DFactory::create(
    VolumetricDataStructure2DType data_structure_type,
    const param::Map& params) {
  const auto config = VolumetricDataStructureConfig::from(params);
  switch (data_structure_type.toTypeId()) {
    case VolumetricDataStructure2DType::kDenseGrid:
      return std::make_shared<DenseGrid<SaturatingOccupancyCell>>(config);
    case VolumetricDataStructure2DType::kHashedBlocks:
      return std::make_shared<HashedBlocks2D<SaturatingOccupancyCell>>(config);
    case VolumetricDataStructure2DType::kQuadtree:
      return std::make_shared<VolumetricQuadtree<SaturatingOccupancyCell>>(
          config);
    case VolumetricDataStructure2DType::kDifferencingQuadtree:
      return std::make_shared<
          VolumetricDifferencingQuadtree<SaturatingOccupancyCell>>(config);
    case VolumetricDataStructure2DType::kWaveletQuadtree:
      return std::make_shared<WaveletQuadtree<SaturatingOccupancyCell>>(config);
    default:
      LOG(ERROR) << "Attempted to create unknown data structure type: "
                 << data_structure_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
