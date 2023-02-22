#include "wavemap/integrator/integrator_factory.h"

#include "wavemap/integrator/integrator_base.h"
#include "wavemap/integrator/measurement_model/constant_ray.h"
#include "wavemap/integrator/measurement_model/continuous_beam.h"
#include "wavemap/integrator/measurement_model/continuous_ray.h"
#include "wavemap/integrator/projection_model/ouster_projector.h"
#include "wavemap/integrator/projection_model/pinhole_camera_projector.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"
#include "wavemap/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap/integrator/projective/coarse_to_fine/wavelet_integrator.h"
#include "wavemap/integrator/projective/fixed_resolution/fixed_resolution_integrator.h"
#include "wavemap/integrator/ray_tracing/ray_tracing_integrator.h"

namespace wavemap {
IntegratorBase::Ptr IntegratorFactory::create(
    const param::Map& params, VolumetricDataStructureBase::Ptr occupancy_map,
    std::optional<IntegratorType> default_integrator_type) {
  std::string error_msg;

  if (param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    const auto& integrator_params =
        param::map::keyGetValue<param::Map>(params, "integration_method");
    auto type = IntegratorType::fromParamMap(integrator_params, error_msg);
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

IntegratorBase::Ptr IntegratorFactory::create(
    IntegratorType integrator_type, const param::Map& params,
    VolumetricDataStructureBase::Ptr occupancy_map) {
  // Load the integrator config
  if (!param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    return nullptr;
  }

  // If we're using a ray tracing based integrator, we're good to go
  if (integrator_type == IntegratorType::kRayTracingIntegrator) {
    const auto integrator_config = RayTracingIntegratorConfig::from(
        param::map::keyGetValue<param::Map>(params, "integration_method"));
    return std::make_shared<RayTracingIntegrator>(integrator_config,
                                                  std::move(occupancy_map));
  }

  // Load the projective integrator's config
  const auto integrator_config = ProjectiveIntegratorConfig::from(
      param::map::keyGetValue<param::Map>(params, "integration_method"));

  // Load the sensor's projection model config
  if (!param::map::keyHoldsValue<param::Map>(params, "projection_model")) {
    return nullptr;
  }
  std::shared_ptr<ProjectorBase> projection_model;
  {
    std::string error_msg;
    const auto& projection_model_params =
        param::map::keyGetValue<param::Map>(params, "projection_model");
    auto projection_model_type =
        ProjectorType::fromParamMap(projection_model_params, error_msg);
    if (!projection_model_type.isValid()) {
      LOG(ERROR) << error_msg;
      return nullptr;
    }
    switch (projection_model_type.toTypeId()) {
      case ProjectorType::kSphericalProjector: {
        const auto spherical_projector_config =
            SphericalProjectorConfig::from(projection_model_params);
        projection_model =
            std::make_shared<SphericalProjector>(spherical_projector_config);
      } break;
      case ProjectorType::kOusterProjector: {
        const auto ouster_projector_config =
            OusterProjectorConfig::from(projection_model_params);
        projection_model =
            std::make_shared<OusterProjector>(ouster_projector_config);
      } break;
      case ProjectorType::kPinholeCameraProjector: {
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
  std::shared_ptr<MeasurementModelBase> measurement_model;
  auto posed_range_image =
      std::make_shared<PosedImage<>>(projection_model->getDimensions());
  auto beam_offset_image =
      std::make_shared<Image<Vector2D>>(projection_model->getDimensions());
  {
    std::string error_msg;
    const auto& measurement_model_params =
        param::map::keyGetValue<param::Map>(params, "measurement_model");
    auto measurement_model_type =
        MeasurementModelType::fromParamMap(measurement_model_params, error_msg);
    if (!measurement_model_type.isValid()) {
      LOG(ERROR) << error_msg;
      return nullptr;
    }
    switch (measurement_model_type.toTypeId()) {
      case MeasurementModelType::kContinuousRay: {
        const auto continuous_ray_config =
            ContinuousRayConfig::from(measurement_model_params);
        measurement_model = std::make_shared<ContinuousRay>(
            continuous_ray_config, projection_model, posed_range_image);
      } break;
      case MeasurementModelType::kContinuousBeam: {
        const auto continuous_beam_config =
            ContinuousBeamConfig::from(measurement_model_params);
        measurement_model = std::make_shared<ContinuousBeam>(
            continuous_beam_config, projection_model, posed_range_image,
            beam_offset_image);
      } break;
    }
  }

  // Assemble the integrator
  switch (integrator_type.toTypeId()) {
    case IntegratorType::kFixedResolutionIntegrator: {
      return std::make_shared<FixedResolutionIntegrator>(
          integrator_config, projection_model, posed_range_image,
          beam_offset_image, measurement_model, std::move(occupancy_map));
    }
    case IntegratorType::kCoarseToFineIntegrator: {
      auto octree_map =
          std::dynamic_pointer_cast<VolumetricOctreeInterface>(occupancy_map);
      return std::make_shared<CoarseToFineIntegrator>(
          integrator_config, projection_model, posed_range_image,
          beam_offset_image, measurement_model, std::move(octree_map));
    }
    case IntegratorType::kWaveletIntegrator: {
      auto wavelet_map =
          std::dynamic_pointer_cast<WaveletOctreeInterface>(occupancy_map);
      return std::make_shared<WaveletIntegrator>(
          integrator_config, projection_model, posed_range_image,
          beam_offset_image, measurement_model, std::move(wavelet_map));
    }
    default:
      LOG(ERROR) << "Attempted to create integrator with unknown type ID: "
                 << integrator_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
