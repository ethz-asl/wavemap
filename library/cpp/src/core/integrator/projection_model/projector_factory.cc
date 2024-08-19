#include "wavemap/core/integrator/projection_model/projector_factory.h"

#include "wavemap/core/integrator/projection_model/ouster_projector.h"
#include "wavemap/core/integrator/projection_model/pinhole_camera_projector.h"
#include "wavemap/core/integrator/projection_model/spherical_projector.h"

namespace wavemap {
std::unique_ptr<ProjectorBase> wavemap::ProjectorFactory::create(
    const param::Value& params,
    std::optional<ProjectorType> default_projector_type) {
  if (const auto type = ProjectorType::from(params, "projection_model"); type) {
    return create(type.value(), params);
  }

  if (default_projector_type.has_value()) {
    LOG(WARNING) << "Default type \"" << default_projector_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_projector_type.value(), params);
  }

  LOG(ERROR) << "No default was set. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<ProjectorBase> wavemap::ProjectorFactory::create(
    ProjectorType projector_type, const param::Value& params) {
  switch (projector_type) {
    case ProjectorType::kSphericalProjector: {
      if (const auto config =
              SphericalProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_unique<SphericalProjector>(config.value());
      } else {
        LOG(ERROR) << "Spherical projector config could not be loaded.";
        return nullptr;
      }
    }
    case ProjectorType::kOusterProjector: {
      if (const auto config =
              OusterProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_unique<OusterProjector>(config.value());
      } else {
        LOG(ERROR) << "Ouster projector config could not be loaded.";
        return nullptr;
      }
    }
    case ProjectorType::kPinholeCameraProjector: {
      if (const auto config =
              PinholeCameraProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_unique<PinholeCameraProjector>(config.value());
      } else {
        LOG(ERROR) << "Pinhole projector config could not be loaded.";
        return nullptr;
      }
    }
  }

  return nullptr;
}
}  // namespace wavemap
