#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d_factory.h"
#include "wavemap_2d/integrator/projective/beamwise_integrator_2d.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_2d.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/wavelet_integrator_2d.h"
#include "wavemap_2d/integrator/projective/fixed_resolution/fixed_resolution_integrator_2d.h"
#include "wavemap_2d/integrator/ray_tracing/ray_integrator_2d.h"

namespace wavemap {
PointcloudIntegrator2D::Ptr PointcloudIntegrator2DFactory::create(
    const param::Map& params, VolumetricDataStructure2D::Ptr occupancy_map,
    std::optional<PointcloudIntegrator2DType> default_integrator_type) {
  std::string error_msg;

  if (param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    const auto& integrator_params =
        param::map::keyGetValue<param::Map>(params, "integration_method");
    auto type =
        PointcloudIntegrator2DType::fromParamMap(integrator_params, error_msg);
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

PointcloudIntegrator2D::Ptr PointcloudIntegrator2DFactory::create(
    PointcloudIntegrator2DType integrator_type, const param::Map& params,
    VolumetricDataStructure2D::Ptr occupancy_map) {
  // Load the integrator config
  if (!param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    return nullptr;
  }
  const auto integrator_config = PointcloudIntegratorConfig::from(
      param::map::keyGetValue<param::Map>(params, "integration_method"));

  // If we're using an integrator type that integrates rays one by one,
  // we're good to go
  if (integrator_type == PointcloudIntegrator2DType::kSingleRayIntegrator) {
    return std::make_shared<RayIntegrator2D>(integrator_config,
                                             std::move(occupancy_map));
  }
  if (integrator_type == PointcloudIntegrator2DType::kSingleBeamIntegrator) {
    return std::make_shared<BeamwiseIntegrator2D>(integrator_config,
                                                  std::move(occupancy_map));
  }

  // Load the sensor's projection model config
  if (!param::map::keyHoldsValue<param::Map>(params, "projection_model")) {
    return nullptr;
  }
  const auto projection_model_config = CircularProjectorConfig::from(
      param::map::keyGetValue<param::Map>(params, "projection_model"));
  const CircularProjector projection_model(projection_model_config);

  // Load the measurement model's config
  if (!param::map::keyHoldsValue<param::Map>(params, "measurement_model")) {
    return nullptr;
  }
  const auto measurement_model_config = ContinuousVolumetricLogOddsConfig::from(
      param::map::keyGetValue<param::Map>(params, "measurement_model"));
  const ContinuousVolumetricLogOdds<2> measurement_model(
      measurement_model_config);

  // Assemble the integrator
  switch (integrator_type.toTypeId()) {
    case PointcloudIntegrator2DType::kSingleRayIntegrator:
    case PointcloudIntegrator2DType::kFixedResolutionScanIntegrator:
      return std::make_shared<FixedResolutionIntegrator2D>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    case PointcloudIntegrator2DType::kCoarseToFineScanIntegrator:
      return std::make_shared<CoarseToFineIntegrator2D>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    case PointcloudIntegrator2DType::kWaveletScanIntegrator:
      return std::make_shared<WaveletIntegrator2D>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    default:
      LOG(ERROR) << "Attempted to create unknown integrator type: "
                 << integrator_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
