#include "wavemap/integrator/pointcloud_integrator_factory.h"

#include "wavemap/integrator/pointcloud_integrator.h"
#include "wavemap/integrator/projection_model/ouster_projector.h"
#include "wavemap/integrator/projection_model/pinhole_camera_projector.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"
#include "wavemap/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap/integrator/projective/coarse_to_fine/wavelet_integrator.h"
#include "wavemap/integrator/projective/fixed_resolution/fixed_resolution_integrator.h"
#include "wavemap/integrator/ray_tracing/ray_integrator.h"

namespace wavemap {
typename PointcloudIntegrator::Ptr PointcloudIntegratorFactory::create(
    const param::Map& params, VolumetricDataStructureBase::Ptr occupancy_map,
    std::optional<PointcloudIntegratorType> default_integrator_type) {
  std::string error_msg;

  if (param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    const auto& integrator_params =
        param::map::keyGetValue<param::Map>(params, "integration_method");
    auto type =
        PointcloudIntegratorType::fromParamMap(integrator_params, error_msg);
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

typename PointcloudIntegrator::Ptr PointcloudIntegratorFactory::create(
    PointcloudIntegratorType integrator_type, const param::Map& params,
    VolumetricDataStructureBase::Ptr occupancy_map) {
  // Load the integrator config
  if (!param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    return nullptr;
  }
  const auto integrator_config = PointcloudIntegratorConfig::from(
      param::map::keyGetValue<param::Map>(params, "integration_method"));

  // If we're using a ray tracing based integrator, we're good to go
  if (integrator_type == PointcloudIntegratorType::kSingleRayIntegrator) {
    return std::make_shared<RayIntegrator>(integrator_config,
                                           std::move(occupancy_map));
  }

  // Load the sensor's projection model config
  if (!param::map::keyHoldsValue<param::Map>(params, "projection_model")) {
    return nullptr;
  }
  std::shared_ptr<Image2DProjectionModel> projection_model;
  {
    std::string error_msg;
    const auto& projection_model_params =
        param::map::keyGetValue<param::Map>(params, "projection_model");
    auto projection_model_type = Image2DProjectionModelType::fromParamMap(
        projection_model_params, error_msg);
    if (!projection_model_type.isValid()) {
      LOG(ERROR) << error_msg;
      return nullptr;
    }
    switch (projection_model_type.toTypeId()) {
      case Image2DProjectionModelType::kSphericalProjector: {
        const auto spherical_projector_config =
            SphericalProjectorConfig::from(projection_model_params);
        projection_model =
            std::make_shared<SphericalProjector>(spherical_projector_config);
      } break;
      case Image2DProjectionModelType::kOusterProjector: {
        const auto ouster_projector_config =
            OusterProjectorConfig::from(projection_model_params);
        projection_model =
            std::make_shared<OusterProjector>(ouster_projector_config);
      } break;
      case Image2DProjectionModelType::kPinholeCameraProjector: {
        const auto pinhole_projector_config =
            PinholeCameraProjectorConfig::from(projection_model_params);
        projection_model =
            std::make_shared<PinholeCameraProjector>(pinhole_projector_config);
      } break;
    }
  }

  // Load the measurement model's config
  if (!param::map::keyHoldsValue<param::Map>(params, "measurement_model")) {
    return nullptr;
  }
  const auto measurement_model_config = ContinuousVolumetricLogOddsConfig::from(
      param::map::keyGetValue<param::Map>(params, "measurement_model"));
  const ContinuousVolumetricLogOdds measurement_model(measurement_model_config);

  // Assemble the integrator
  switch (integrator_type.toTypeId()) {
    case PointcloudIntegratorType::kFixedResolutionScanIntegrator:
      return std::make_shared<FixedResolutionIntegrator>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    case PointcloudIntegratorType::kCoarseToFineScanIntegrator:
      return std::make_shared<CoarseToFineIntegrator>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    case PointcloudIntegratorType::kWaveletScanIntegrator:
      return std::make_shared<WaveletIntegrator>(
          integrator_config, projection_model, measurement_model,
          std::move(occupancy_map));
    default:
      LOG(ERROR) << "Attempted to create integrator with unknown type ID: "
                 << integrator_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
