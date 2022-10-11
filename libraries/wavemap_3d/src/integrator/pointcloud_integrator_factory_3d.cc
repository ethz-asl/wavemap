#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d_factory.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_3d.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/wavelet_integrator_3d.h"
#include "wavemap_3d/integrator/projective/fixed_resolution/fixed_resolution_integrator_3d.h"
#include "wavemap_3d/integrator/ray_tracing/ray_integrator_3d.h"

namespace wavemap {
typename PointcloudIntegrator3D::Ptr PointcloudIntegrator3DFactory::create(
    const param::Map& params, VolumetricDataStructure3D::Ptr occupancy_map,
    std::optional<PointcloudIntegrator3DType> default_integrator_type) {
  std::string error_msg;
  auto type = PointcloudIntegrator3DType::fromParamMap(params, error_msg);
  if (type.isValid()) {
    return create(type, params, std::move(occupancy_map));
  }

  if (default_integrator_type.has_value()) {
    type = default_integrator_type.value();
    LOG(WARNING) << error_msg << " Default type \"" << type.toStr()
                 << "\" will be created instead.";
    return create(default_integrator_type.value(), params,
                  std::move(occupancy_map));
  }

  LOG(ERROR) << error_msg << " No default was set. Returning nullptr.";
  return nullptr;
}

typename PointcloudIntegrator3D::Ptr PointcloudIntegrator3DFactory::create(
    PointcloudIntegrator3DType integrator_type, const param::Map& /*params*/,
    VolumetricDataStructure3D::Ptr occupancy_map) {
  switch (integrator_type.toTypeId()) {
    case PointcloudIntegrator3DType::kSingleRayIntegrator:
      return std::make_shared<RayIntegrator3D>(std::move(occupancy_map));
    case PointcloudIntegrator3DType::kFixedResolutionScanIntegrator:
      return std::make_shared<FixedResolutionIntegrator3D>(
          std::move(occupancy_map));
    case PointcloudIntegrator3DType::kCoarseToFineScanIntegrator:
      return std::make_shared<CoarseToFineIntegrator3D>(
          std::move(occupancy_map));
    case PointcloudIntegrator3DType::kWaveletScanIntegrator:
      return std::make_shared<WaveletIntegrator3D>(std::move(occupancy_map));
    default:
      LOG(ERROR) << "Attempted to create integrator with unknown type ID: "
                 << integrator_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
