#include "wavemap/integrator/projection_model/projector_factory.h"

#include "wavemap/integrator/projection_model/ouster_projector.h"
#include "wavemap/integrator/projection_model/pinhole_camera_projector.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"

namespace wavemap {
ProjectorBase::Ptr wavemap::ProjectorFactory::create(
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

ProjectorBase::Ptr wavemap::ProjectorFactory::create(
    ProjectorType projector_type, const param::Value& params) {
  switch (projector_type.toTypeId()) {
    case ProjectorType::kSphericalProjector: {
      if (const auto config =
              SphericalProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_shared<SphericalProjector>(config.value());
      } else {
        LOG(ERROR) << "Spherical projector config could not be loaded.";
        return nullptr;
      }
    }
    case ProjectorType::kOusterProjector: {
      if (const auto config =
              OusterProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_shared<OusterProjector>(config.value());
      } else {
        LOG(ERROR) << "Ouster projector config could not be loaded.";
        return nullptr;
      }
    }
    case ProjectorType::kPinholeCameraProjector: {
      if (const auto config =
              PinholeCameraProjectorConfig::from(params, "projection_model");
          config) {
        return std::make_shared<PinholeCameraProjector>(config.value());
      } else {
        LOG(ERROR) << "Pinhole projector config could not be loaded.";
        return nullptr;
      }
    }
  }

  return nullptr;
}
}  // namespace wavemap
