#ifndef WAVEMAP_CORE_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_FACTORY_H_
#define WAVEMAP_CORE_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_FACTORY_H_

#include <memory>

#include "wavemap/core/integrator/measurement_model/measurement_model_base.h"

namespace wavemap {
class MeasurementModelFactory {
 public:
  static std::unique_ptr<MeasurementModelBase> create(
      const param::Value& params, ProjectorBase::ConstPtr projection_model,
      Image<>::ConstPtr range_image,
      Image<Vector2D>::ConstPtr beam_offset_image = nullptr,
      std::optional<MeasurementModelType> default_measurement_model_type =
          std::nullopt);

  static std::unique_ptr<MeasurementModelBase> create(
      MeasurementModelType measurement_model_type,
      ProjectorBase::ConstPtr projection_model, Image<>::ConstPtr range_image,
      Image<Vector2D>::ConstPtr beam_offset_image, const param::Value& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_FACTORY_H_
