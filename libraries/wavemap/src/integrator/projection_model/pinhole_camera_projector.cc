#include "wavemap/integrator/projection_model/pinhole_camera_projector.h"

namespace wavemap {
bool PinholeCameraProjectorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(width, 0, verbose);
  is_valid &= IS_PARAM_GT(height, 0, verbose);

  is_valid &= IS_PARAM_GT(fx, 0.f, verbose);
  is_valid &= IS_PARAM_GT(fy, 0.f, verbose);

  return is_valid;
}

PinholeCameraProjectorConfig PinholeCameraProjectorConfig::from(
    const param::Map& params) {
  PinholeCameraProjectorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(width)) {
      config.width = param_value.get<IndexElement>();
    } else if (param_name == NAMEOF(height)) {
      config.height = param_value.get<IndexElement>();
    } else if (param_name == NAMEOF(fx)) {
      config.fx = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(fy)) {
      config.fy = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(cx)) {
      config.cx = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(cy)) {
      config.cy = param_value.get<FloatingPoint>();
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
