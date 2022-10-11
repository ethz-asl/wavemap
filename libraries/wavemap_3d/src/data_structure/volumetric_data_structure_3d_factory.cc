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
  std::ostringstream error_msg;

  error_msg << "Requested creation of ";
  if (param::map::hasKey(params, "type")) {
    if (param::map::keyHoldsValue<std::string>(params, "type")) {
      const std::string& data_structure_type_name =
          params.at("type").get<std::string>();
      for (size_t type_idx = 0;
           type_idx < kVolumetricDataStructure3DTypeStrs.size(); ++type_idx) {
        if (data_structure_type_name ==
            kVolumetricDataStructure3DTypeStrs[type_idx]) {
          return create(static_cast<VolumetricDataStructure3DType>(type_idx),
                        params);
        }
      }
      error_msg << "unknown data structure type. ";
    } else {
      error_msg << "data structure but specified type param is not a string. ";
    }
  } else {
    error_msg << "data structure without specifying the desired type. ";
  }

  if (default_data_structure_type.has_value()) {
    const std::string default_type_str = getVolumetricDataStructure3DTypeStr(
        default_data_structure_type.value());
    LOG(WARNING) << error_msg.str() << "Default type \"" << default_type_str
                 << "\" will be created instead.";
    param::Map params_corrected = params;
    params_corrected.erase("type");
    params_corrected.emplace("type", default_type_str);
    return create(default_data_structure_type.value(), params_corrected);
  }

  LOG(ERROR) << error_msg.str() << "No default was set. Returning nullptr.";
  return nullptr;
}

VolumetricDataStructure3D::Ptr VolumetricDataStructure3DFactory::create(
    VolumetricDataStructure3DType data_structure_type,
    const param::Map& params) {
  const auto config = VolumetricDataStructure3D::Config::fromParams(params);
  switch (data_structure_type) {
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
      LOG(ERROR) << "Attempted to create unknown data structure type: "
                 << to_underlying(data_structure_type)
                 << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
