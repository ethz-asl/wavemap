#include "wavemap_3d/integrator/pointcloud_integrator_factory.h"

#include "wavemap_3d/integrator/point_integrator/ray_integrator.h"

namespace wavemap {
PointcloudIntegrator::Ptr PointcloudIntegratorFactory::create(
    const std::string& integrator_type_name,
    VolumetricDataStructure3D::Ptr occupancy_map,
    std::optional<PointcloudIntegratorType> default_integrator_type) {
  for (size_t type_idx = 0; type_idx < kPointcloudIntegratorTypeStrs.size();
       ++type_idx) {
    if (integrator_type_name == kPointcloudIntegratorTypeStrs[type_idx]) {
      return create(static_cast<PointcloudIntegratorType>(type_idx),
                    std::move(occupancy_map));
    }
  }

  if (default_integrator_type.has_value()) {
    LOG(WARNING) << "Requested creation of unknown integrator type \""
                 << integrator_type_name << "\". Default type \""
                 << getPointcloudIntegratorTypeStr(
                        default_integrator_type.value())
                 << "\" will be created instead.";
    return create(default_integrator_type.value(), std::move(occupancy_map));
  } else {
    LOG(ERROR) << "Requested creation of unknown integrator type \""
               << integrator_type_name
               << "\" and no default was set. Returning nullptr.";
  }
  return nullptr;
}

PointcloudIntegrator::Ptr PointcloudIntegratorFactory::create(
    PointcloudIntegratorType integrator_type,
    VolumetricDataStructure3D::Ptr occupancy_map) {
  switch (integrator_type) {
    case PointcloudIntegratorType::kSingleRayIntegrator:
      return std::make_shared<RayIntegrator>(std::move(occupancy_map));
    default:
      LOG(ERROR) << "Attempted to create unknown integrator type: "
                 << to_underlying(integrator_type) << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
