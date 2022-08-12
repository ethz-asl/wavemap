#include <wavemap_common/integrator/ray_tracing/ray_integrator.h>

#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d_factory.h"
#include "wavemap_2d/integrator/projective/beamwise_integrator_2d.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_2d.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/wavelet_integrator_2d.h"
#include "wavemap_2d/integrator/projective/fixed_resolution/fixed_resolution_integrator_2d.h"

namespace wavemap {
PointcloudIntegrator2D::Ptr PointcloudIntegrator2DFactory::create(
    const std::string& integrator_type_name,
    VolumetricDataStructure2D::Ptr occupancy_map,
    std::optional<PointcloudIntegrator2DType> default_integrator_type) {
  for (size_t type_idx = 0; type_idx < kPointcloudIntegrator2DTypeStrs.size();
       ++type_idx) {
    if (integrator_type_name == kPointcloudIntegrator2DTypeStrs[type_idx]) {
      return create(static_cast<PointcloudIntegrator2DType>(type_idx),
                    std::move(occupancy_map));
    }
  }

  if (default_integrator_type.has_value()) {
    LOG(WARNING) << "Requested creation of unknown integrator type \""
                 << integrator_type_name << "\". Default type \""
                 << getPointcloudIntegrator2DTypeStr(
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

PointcloudIntegrator2D::Ptr PointcloudIntegrator2DFactory::create(
    PointcloudIntegrator2DType integrator_type,
    VolumetricDataStructure2D::Ptr occupancy_map) {
  switch (integrator_type) {
    case PointcloudIntegrator2DType::kSingleRayIntegrator:
      return std::make_shared<RayIntegrator<2>>(std::move(occupancy_map));
    case PointcloudIntegrator2DType::kSingleBeamIntegrator:
      return std::make_shared<BeamwiseIntegrator2D>(std::move(occupancy_map));
    case PointcloudIntegrator2DType::kFixedResolutionScanIntegrator:
      return std::make_shared<FixedResolutionIntegrator2D>(
          std::move(occupancy_map));
    case PointcloudIntegrator2DType::kCoarseToFineScanIntegrator:
      return std::make_shared<CoarseToFineIntegrator2D>(
          std::move(occupancy_map));
    case PointcloudIntegrator2DType::kWaveletScanIntegrator:
      return std::make_shared<WaveletIntegrator2D>(std::move(occupancy_map));
    default:
      LOG(ERROR) << "Attempted to create unknown integrator type: "
                 << to_underlying(integrator_type) << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
