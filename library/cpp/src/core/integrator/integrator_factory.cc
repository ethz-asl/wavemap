#include "wavemap/core/integrator/integrator_factory.h"

#include <memory>
#include <utility>

#include "wavemap/core/integrator/integrator_base.h"
#include "wavemap/core/integrator/measurement_model/measurement_model_factory.h"
#include "wavemap/core/integrator/projection_model/projector_factory.h"
#include "wavemap/core/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap/core/integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.h"
#include "wavemap/core/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h"
#include "wavemap/core/integrator/projective/coarse_to_fine/wavelet_integrator.h"
#include "wavemap/core/integrator/projective/fixed_resolution/fixed_resolution_integrator.h"
#include "wavemap/core/integrator/ray_tracing/ray_tracing_integrator.h"

namespace wavemap {
std::unique_ptr<IntegratorBase> IntegratorFactory::create(
    const param::Value& params, MapBase::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool,
    std::optional<IntegratorType> default_integrator_type) {
  if (const auto type = IntegratorType::from(params, "integration_method");
      type) {
    return create(type.value(), params, std::move(occupancy_map),
                  std::move(thread_pool));
  }

  if (default_integrator_type.has_value()) {
    LOG(WARNING) << "Default type \"" << default_integrator_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_integrator_type.value(), params,
                  std::move(occupancy_map), std::move(thread_pool));
  }

  LOG(ERROR) << "No default was set. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<IntegratorBase> IntegratorFactory::create(
    IntegratorType integrator_type, const param::Value& params,
    MapBase::Ptr occupancy_map, std::shared_ptr<ThreadPool> thread_pool) {
  // If we're using a ray tracing based integrator, we can build it directly
  if (integrator_type == IntegratorType::kRayTracingIntegrator) {
    if (const auto config =
            RayTracingIntegratorConfig::from(params, "integration_method");
        config) {
      return std::make_unique<RayTracingIntegrator>(config.value(),
                                                    std::move(occupancy_map));
    } else {
      LOG(ERROR) << "Ray tracing integrator config could not be loaded.";
      return nullptr;
    }
  }

  // Load the projective integrator config
  const auto integrator_config =
      ProjectiveIntegratorConfig::from(params, "integration_method");
  if (!integrator_config.has_value()) {
    LOG(ERROR) << "Integrator config could not be loaded.";
    return nullptr;
  }

  // Create the projection model
  const auto projector_params = params.getChild("projection_model");
  if (!projector_params) {
    LOG(ERROR) << "No params with key \"projection_model\". "
                  "Projection model could not be created.";
    return nullptr;
  }
  std::shared_ptr<ProjectorBase> projection_model =
      ProjectorFactory::create(projector_params.value());
  if (!projection_model) {
    LOG(ERROR) << "Creating projection model failed.";
    return nullptr;
  }

  // Create the range and beam-offset images
  // NOTE: These are shared by the integrator and measurement model
  auto posed_range_image =
      std::make_shared<PosedImage<>>(projection_model->getDimensions());
  auto beam_offset_image =
      std::make_shared<Image<Vector2D>>(projection_model->getDimensions());

  // Create the measurement model
  const auto measurement_params = params.getChild("measurement_model");
  if (!measurement_params) {
    LOG(ERROR) << "No params with key \"measurement_model\". "
                  "Measurement model could not be created.";
    return nullptr;
  }
  std::shared_ptr<MeasurementModelBase> measurement_model =
      MeasurementModelFactory::create(measurement_params.value(),
                                      projection_model, posed_range_image,
                                      beam_offset_image);
  if (!measurement_model) {
    LOG(ERROR) << "Creating measurement model failed.";
    return nullptr;
  }

  // Assemble the projective integrator
  switch (integrator_type) {
    case IntegratorType::kFixedResolutionIntegrator: {
      return std::make_unique<FixedResolutionIntegrator>(
          integrator_config.value(), projection_model, posed_range_image,
          beam_offset_image, measurement_model, std::move(occupancy_map));
    }
    case IntegratorType::kCoarseToFineIntegrator: {
      auto octree_map =
          std::dynamic_pointer_cast<VolumetricOctree>(occupancy_map);
      if (octree_map) {
        return std::make_unique<CoarseToFineIntegrator>(
            integrator_config.value(), projection_model, posed_range_image,
            beam_offset_image, measurement_model, std::move(octree_map));
      } else {
        LOG(ERROR) << "Integrator of type " << integrator_type.toStr()
                   << " only supports data structures of type "
                   << MapType::toStr(MapType::kOctree)
                   << ". Returning nullptr.";
      }
      break;
    }
    case IntegratorType::kWaveletIntegrator: {
      auto wavelet_map =
          std::dynamic_pointer_cast<WaveletOctree>(occupancy_map);
      if (wavelet_map) {
        return std::make_unique<WaveletIntegrator>(
            integrator_config.value(), projection_model, posed_range_image,
            beam_offset_image, measurement_model, std::move(wavelet_map));
      } else {
        LOG(ERROR) << "Integrator of type " << integrator_type.toStr()
                   << " only supports data structures of type "
                   << MapType::toStr(MapType::kWaveletOctree)
                   << ". Returning nullptr.";
      }
      break;
    }
    case IntegratorType::kHashedWaveletIntegrator: {
      auto hashed_wavelet_map =
          std::dynamic_pointer_cast<HashedWaveletOctree>(occupancy_map);
      if (hashed_wavelet_map) {
        return std::make_unique<HashedWaveletIntegrator>(
            integrator_config.value(), projection_model, posed_range_image,
            beam_offset_image, measurement_model, std::move(hashed_wavelet_map),
            std::move(thread_pool));
      } else {
        LOG(ERROR) << "Integrator of type " << integrator_type.toStr()
                   << " only supports data structures of type "
                   << MapType::toStr(MapType::kHashedWaveletOctree)
                   << ". Returning nullptr.";
      }
      break;
    }
    case IntegratorType::kHashedChunkedWaveletIntegrator: {
      auto hashed_chunked_wavelet_map =
          std::dynamic_pointer_cast<HashedChunkedWaveletOctree>(occupancy_map);
      if (hashed_chunked_wavelet_map) {
        return std::make_unique<HashedChunkedWaveletIntegrator>(
            integrator_config.value(), projection_model, posed_range_image,
            beam_offset_image, measurement_model,
            std::move(hashed_chunked_wavelet_map), std::move(thread_pool));
      } else {
        LOG(ERROR) << "Integrator of type " << integrator_type.toStr()
                   << " only supports data structures of type "
                   << MapType::toStr(MapType::kHashedChunkedWaveletOctree)
                   << ". Returning nullptr.";
      }
      break;
    }
    default:
      LOG(ERROR) << "Attempted to create integrator with unknown type ID: "
                 << integrator_type.toTypeId() << ". Returning nullptr.";
      break;
  }
  return nullptr;
}
}  // namespace wavemap
