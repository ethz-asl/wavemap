#include "wavemap/integrator/integrator_factory.h"

#include "wavemap/data_structure/volumetric/volumetric_octree.h"
#include "wavemap/data_structure/volumetric/wavelet_octree.h"
#include "wavemap/integrator/integrator_base.h"
#include "wavemap/integrator/measurement_model/measurement_model_factory.h"
#include "wavemap/integrator/projection_model/projector_factory.h"
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
  // If we're using a ray tracing based integrator, we can build it directly
  if (integrator_type == IntegratorType::kRayTracingIntegrator) {
    if (!param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
      return nullptr;
    }
    const auto integrator_config = RayTracingIntegratorConfig::from(
        param::map::keyGetValue<param::Map>(params, "integration_method"));
    return std::make_shared<RayTracingIntegrator>(integrator_config,
                                                  std::move(occupancy_map));
  }

  // Load the projective integrator config
  if (!param::map::keyHoldsValue<param::Map>(params, "integration_method")) {
    return nullptr;
  }
  const auto integrator_config = ProjectiveIntegratorConfig::from(
      param::map::keyGetValue<param::Map>(params, "integration_method"));

  // Create the projection model
  std::shared_ptr<ProjectorBase> projection_model =
      ProjectorFactory::create(params);

  // Create the range and beam-offset images
  // NOTE: These are shared by the integrator and measurement model
  auto posed_range_image =
      std::make_shared<PosedImage<>>(projection_model->getDimensions());
  auto beam_offset_image =
      std::make_shared<Image<Vector2D>>(projection_model->getDimensions());

  // Create the measurement model
  std::shared_ptr<MeasurementModelBase> measurement_model =
      MeasurementModelFactory::create(params, projection_model,
                                      posed_range_image, beam_offset_image);

  // Assemble the projective integrator
  switch (integrator_type.toTypeId()) {
    case IntegratorType::kFixedResolutionIntegrator: {
      return std::make_shared<FixedResolutionIntegrator>(
          integrator_config, projection_model, posed_range_image,
          beam_offset_image, measurement_model, std::move(occupancy_map));
    }
    case IntegratorType::kCoarseToFineIntegrator: {
      auto octree_map =
          std::dynamic_pointer_cast<VolumetricOctree>(occupancy_map);
      return std::make_shared<CoarseToFineIntegrator>(
          integrator_config, projection_model, posed_range_image,
          beam_offset_image, measurement_model, std::move(octree_map));
    }
    case IntegratorType::kWaveletIntegrator: {
      auto wavelet_map =
          std::dynamic_pointer_cast<WaveletOctree>(occupancy_map);
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
