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

  if (param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    const auto& integrator_params =
        param::map::keyGetValue<param::Map>(params, "integration_method");
    auto type =
        PointcloudIntegrator3DType::fromParamMap(integrator_params, error_msg);
    if (type.isValid()) {
      return create(type, params, std::move(occupancy_map));
    }
  }

  if (default_integrator_type.has_value()) {
    LOG(WARNING) << error_msg << " Default type \""
                 << default_integrator_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_integrator_type.value(), params,
                  std::move(occupancy_map));
  }

  LOG(ERROR) << error_msg << " No default was set. Returning nullptr.";
  return nullptr;
}

typename PointcloudIntegrator3D::Ptr PointcloudIntegrator3DFactory::create(
    PointcloudIntegrator3DType integrator_type, const param::Map& params,
    VolumetricDataStructure3D::Ptr occupancy_map) {
  // Load the integrator config
  if (!param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    return nullptr;
  }
  const auto integrator_config = PointcloudIntegratorConfig::from(
      param::map::keyGetValue<param::Map>(params, "integration_method"));

  // If we're using a ray tracing based integrator, we're good to go
  if (integrator_type == PointcloudIntegrator3DType::kSingleRayIntegrator) {
    return std::make_shared<RayIntegrator3D>(integrator_config,
                                             std::move(occupancy_map));
  }

  // Load the sensor's projection model config
  if (!param::map::keyHoldsValue<param::Map>(params, "projection_model")) {
    return nullptr;
  }
  const auto projection_model_config = SphericalProjectorConfig::from(
      param::map::keyGetValue<param::Map>(params, "projection_model"));
  const SphericalProjector projection_model(projection_model_config);

  // Load the measurement model's config
  if (!param::map::keyHoldsValue<param::Map>(params, "measurement_model")) {
    return nullptr;
  }
  const auto measurement_model_config = ContinuousVolumetricLogOddsConfig::from(
      param::map::keyGetValue<param::Map>(params, "measurement_model"));
  const ContinuousVolumetricLogOdds<3> measurement_model(
      measurement_model_config);

  // Assemble the integrator
  switch (integrator_type.toTypeId()) {
    case PointcloudIntegrator3DType::kFixedResolutionScanIntegrator:
      return std::make_shared<FixedResolutionIntegrator3D>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    case PointcloudIntegrator3DType::kCoarseToFineScanIntegrator:
      return std::make_shared<CoarseToFineIntegrator3D>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    case PointcloudIntegrator3DType::kWaveletScanIntegrator:
      return std::make_shared<WaveletIntegrator3D>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    default:
      LOG(ERROR) << "Attempted to create integrator with unknown type ID: "
                 << integrator_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
