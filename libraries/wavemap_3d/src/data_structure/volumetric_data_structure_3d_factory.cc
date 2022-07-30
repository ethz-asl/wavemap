#include "wavemap_3d/data_structure/volumetric_data_structure_3d_factory.h"

#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>

#include "wavemap_3d/data_structure/hashed_blocks_3d.h"

namespace wavemap {
VolumetricDataStructure3D::Ptr VolumetricDataStructure3DFactory::create(
    const std::string& data_structure_type_name, FloatingPoint min_cell_width,
    std::optional<VolumetricDataStructure3DType> default_data_structure_type) {
  for (size_t type_idx = 0;
       type_idx < kVolumetricDataStructure3DTypeStrs.size(); ++type_idx) {
    if (data_structure_type_name ==
        kVolumetricDataStructure3DTypeStrs[type_idx]) {
      return create(static_cast<VolumetricDataStructure3DType>(type_idx),
                    min_cell_width);
    }
  }

  if (default_data_structure_type.has_value()) {
    LOG(WARNING) << "Requested creation of unknown data structure type \""
                 << data_structure_type_name << "\". Default type \""
                 << getVolumetricDataStructure3DTypeStr(
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

VolumetricDataStructure3D::Ptr VolumetricDataStructure3DFactory::create(
    VolumetricDataStructure3DType data_structure_type,
    FloatingPoint min_cell_width) {
  switch (data_structure_type) {
    case VolumetricDataStructure3DType::kHashedBlocks:
      return std::make_shared<HashedBlocks3D<SaturatingOccupancyCell>>(
          min_cell_width);
    default:
      LOG(ERROR) << "Attempted to create unknown data structure type: "
                 << to_underlying(data_structure_type)
                 << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
