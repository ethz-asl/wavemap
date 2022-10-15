#include "wavemap_3d/data_structure/volumetric_data_structure_3d_factory.h"

#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>

#include "wavemap_3d/data_structure/hashed_blocks_3d.h"
#include "wavemap_3d/data_structure/volumetric_differencing_octree.h"
#include "wavemap_3d/data_structure/volumetric_octree.h"
#include "wavemap_3d/data_structure/wavelet_octree.h"

namespace wavemap {
VolumetricDataStructure3D::Ptr VolumetricDataStructure3DFactory::create(
    const param::Map& params,
    std::optional<VolumetricDataStructure3DType> default_data_structure_type) {
  std::string error_msg;
  auto type = VolumetricDataStructure3DType::fromParamMap(params, error_msg);
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

VolumetricDataStructure3D::Ptr VolumetricDataStructure3DFactory::create(
    VolumetricDataStructure3DType data_structure_type,
    const param::Map& params) {
  const auto config = VolumetricDataStructureConfig::from(params);
  switch (data_structure_type.toTypeId()) {
    case VolumetricDataStructure3DType::kHashedBlocks:
      return std::make_shared<HashedBlocks3D<SaturatingOccupancyCell>>(config);
    case VolumetricDataStructure3DType::kOctree:
      return std::make_shared<VolumetricOctree<SaturatingOccupancyCell>>(
          config);
    case VolumetricDataStructure3DType::kDifferencingOctree:
      return std::make_shared<
          VolumetricDifferencingOctree<SaturatingOccupancyCell>>(config);
    case VolumetricDataStructure3DType::kWaveletOctree:
      return std::make_shared<WaveletOctree<SaturatingOccupancyCell>>(config);
    default:
      LOG(ERROR) << "Attempted to create data structure with unknown type ID: "
                 << data_structure_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
