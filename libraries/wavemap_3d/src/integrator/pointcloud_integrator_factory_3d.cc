#include <wavemap_common/integrator/ray_tracing/ray_integrator.h>

#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d_factory.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_3d.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/wavelet_integrator_3d.h"
#include "wavemap_3d/integrator/projective/fixed_resolution/fixed_resolution_integrator_3d.h"

namespace wavemap {
typename PointcloudIntegrator3D::Ptr PointcloudIntegrator3DFactory::create(
    const param::Map& params, VolumetricDataStructure3D::Ptr occupancy_map,
    std::optional<PointcloudIntegrator3DType> default_integrator_type) {
  std::ostringstream error_msg;

  error_msg << "Requested creation of ";
  if (param::map::hasKey(params, "type")) {
    if (param::map::keyHoldsValue<std::string>(params, "type")) {
      const std::string& integrator_type_name =
          params.at("type").get<std::string>();
      for (size_t type_idx = 0;
           type_idx < kPointcloudIntegrator3DTypeStrs.size(); ++type_idx) {
        if (integrator_type_name == kPointcloudIntegrator3DTypeStrs[type_idx]) {
          return create(static_cast<PointcloudIntegrator3DType>(type_idx),
                        params, std::move(occupancy_map));
        }
      }
      error_msg << "unknown integrator type. ";
    } else {
      error_msg << "integrator but specified type param is not a string. ";
    }
  } else {
    error_msg << "integrator without specifying the desired type. ";
  }

  if (default_integrator_type.has_value()) {
    const std::string default_type_str =
        getPointcloudIntegrator3DTypeStr(default_integrator_type.value());
    LOG(WARNING) << error_msg.str() << "Default type \"" << default_type_str
                 << "\" will be created instead.";
    param::Map params_corrected = params;
    params_corrected.erase("type");
    params_corrected.emplace("type", default_type_str);
    return create(default_integrator_type.value(), params_corrected,
                  std::move(occupancy_map));
  }

  LOG(ERROR) << error_msg.str() << "No default was set. Returning nullptr.";
  return nullptr;
}

typename PointcloudIntegrator3D::Ptr PointcloudIntegrator3DFactory::create(
    PointcloudIntegrator3DType integrator_type, const param::Map& /*params*/,
    VolumetricDataStructure3D::Ptr occupancy_map) {
  switch (integrator_type) {
    case PointcloudIntegrator3DType::kSingleRayIntegrator:
      return std::make_shared<RayIntegrator<3>>(std::move(occupancy_map));
    case PointcloudIntegrator3DType::kFixedResolutionScanIntegrator:
      return std::make_shared<FixedResolutionIntegrator3D>(
          std::move(occupancy_map));
    case PointcloudIntegrator3DType::kCoarseToFineScanIntegrator:
      return std::make_shared<CoarseToFineIntegrator3D>(
          std::move(occupancy_map));
    case PointcloudIntegrator3DType::kWaveletScanIntegrator:
      return std::make_shared<WaveletIntegrator3D>(std::move(occupancy_map));
    default:
      LOG(ERROR) << "Attempted to create unknown integrator type: "
                 << to_underlying(integrator_type) << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
