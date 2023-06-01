#include "wavemap/integrator/projection_model/projector_factory.h"

#include "wavemap/integrator/projection_model/ouster_projector.h"
#include "wavemap/integrator/projection_model/pinhole_camera_projector.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"

namespace wavemap {
ProjectorBase::Ptr wavemap::ProjectorFactory::create(
    const param::Map& params,
    std::optional<ProjectorType> default_projector_type) {
  std::string error_msg;

  if (param::map::keyHoldsValue<param::Map>(params, "projection_model")) {
    const auto& projection_model_params =
        param::map::keyGetValue<param::Map>(params, "projection_model");
    auto type = ProjectorType::fromParamMap(projection_model_params, error_msg);
    if (type.isValid()) {
      return create(type, params);
    }
  }

  if (default_projector_type.has_value()) {
    LOG(WARNING) << error_msg << " Default type \""
                 << default_projector_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_projector_type.value(), params);
  }

  LOG(ERROR) << error_msg << " No default was set. Returning nullptr.";
  return nullptr;
}

ProjectorBase::Ptr wavemap::ProjectorFactory::create(
    ProjectorType projector_type, const param::Map& params) {
  const auto& projection_model_params =
      param::map::keyGetValue<param::Map>(params, "projection_model");
  switch (projector_type.toTypeId()) {
    case ProjectorType::kSphericalProjector: {
      const auto spherical_projector_config =
          SphericalProjectorConfig::from(projection_model_params);
      return std::make_shared<SphericalProjector>(spherical_projector_config);
    }
    case ProjectorType::kOusterProjector: {
      const auto ouster_projector_config =
          OusterProjectorConfig::from(projection_model_params);
      return std::make_shared<OusterProjector>(ouster_projector_config);
    }
    case ProjectorType::kPinholeCameraProjector: {
      const auto pinhole_projector_config =
          PinholeCameraProjectorConfig::from(projection_model_params);
      return std::make_shared<PinholeCameraProjector>(pinhole_projector_config);
    }
  }

  return nullptr;
}
}  // namespace wavemap
