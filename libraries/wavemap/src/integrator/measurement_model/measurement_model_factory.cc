#include "wavemap/integrator/measurement_model/measurement_model_factory.h"

#include "wavemap/integrator/measurement_model/continuous_beam.h"
#include "wavemap/integrator/measurement_model/continuous_ray.h"

namespace wavemap {
MeasurementModelBase::Ptr wavemap::MeasurementModelFactory::create(
    const param::Map& params, ProjectorBase::ConstPtr projection_model,
    Image<>::ConstPtr range_image, Image<Vector2D>::ConstPtr beam_offset_image,
    std::optional<MeasurementModelType> default_measurement_model_type) {
  std::string error_msg;

  if (param::map::keyHoldsValue<param::Map>(params, "measurement_model")) {
    const auto& measurement_model_params =
        param::map::keyGetValue<param::Map>(params, "measurement_model");
    auto type =
        MeasurementModelType::fromParamMap(measurement_model_params, error_msg);
    if (type.isValid()) {
      return create(type, std::move(projection_model), std::move(range_image),
                    std::move(beam_offset_image), params);
    }
  }

  if (default_measurement_model_type.has_value()) {
    LOG(WARNING) << error_msg << " Default type \""
                 << default_measurement_model_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_measurement_model_type.value(),
                  std::move(projection_model), std::move(range_image),
                  std::move(beam_offset_image), params);
  }

  LOG(ERROR) << error_msg << " No default was set. Returning nullptr.";
  return nullptr;
}

MeasurementModelBase::Ptr wavemap::MeasurementModelFactory::create(
    MeasurementModelType measurement_model_type,
    ProjectorBase::ConstPtr projection_model, Image<>::ConstPtr range_image,
    Image<Vector2D>::ConstPtr beam_offset_image, const param::Map& params) {
  const auto& measurement_model_params =
      param::map::keyGetValue<param::Map>(params, "measurement_model");
  switch (measurement_model_type.toTypeId()) {
    case MeasurementModelType::kContinuousRay: {
      const auto continuous_ray_config =
          ContinuousRayConfig::from(measurement_model_params);
      if (continuous_ray_config.has_value()) {
        return std::make_shared<ContinuousRay>(continuous_ray_config.value(),
                                               std::move(projection_model),
                                               std::move(range_image));
      } else {
        LOG(ERROR)
            << "Continuous ray measurement model config could not be loaded.";
        return nullptr;
      }
    }
    case MeasurementModelType::kContinuousBeam: {
      const auto continuous_beam_config =
          ContinuousBeamConfig::from(measurement_model_params);
      if (continuous_beam_config.has_value()) {
        return std::make_shared<ContinuousBeam>(
            continuous_beam_config.value(), std::move(projection_model),
            std::move(range_image), std::move(beam_offset_image));
      } else {
        LOG(ERROR)
            << "Continuous beam measurement model config could not be loaded.";
        return nullptr;
      }
    }
  }

  return nullptr;
}
}  // namespace wavemap
