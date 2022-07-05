#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_

#include <wavemap_common/common.h>

#include "wavemap_2d/integrator/measurement_model/measurement_model.h"

namespace wavemap {
class BeamModel : public MeasurementModel {
 public:
  static constexpr FloatingPoint kAngleSigma = kPi / 400.f / 2.f / 6.f;
  static constexpr FloatingPoint kRangeSigma = 0.15f / 6.f;
  static constexpr FloatingPoint kAngleThresh = 6.f * kAngleSigma;
  static constexpr FloatingPoint kRangeDeltaThresh = 6.f * kRangeSigma;
  static constexpr FloatingPoint kScaling = 0.5f;
  // NOTE: The angle and upper range thresholds have a width of 6 sigmas because
  //       the ground truth surface thickness is 3 sigma, and the angular/range
  //       uncertainty extends the non-zero regions with another 3 sigma.

  // Use the base class' constructor
  using MeasurementModel::MeasurementModel;

  Index getBottomLeftUpdateIndex() const override;
  Index getTopRightUpdateIndex() const override;

  FloatingPoint computeUpdateAt(const Index& index) const override;
  static FloatingPoint computeUpdate(FloatingPoint cell_to_sensor_distance,
                                     FloatingPoint cell_to_beam_angle,
                                     FloatingPoint measured_distance);

 private:
  FloatingPoint beam_angle_;
  FloatingPoint max_lateral_component_ = 0.f;

  void updateCachedVariablesDerived() override {
    if (measured_distance_ < kEpsilon) {
      beam_angle_ = 0.f;
    } else {
      const Point C_end_point = W_end_point_ - W_start_point_;
      beam_angle_ = std::atan2(C_end_point.y(), C_end_point.x());
    }
    // TODO(victorr): Calculate this properly
    max_lateral_component_ = std::max(
        std::sin(kAngleThresh) * (measured_distance_ + kRangeDeltaThresh),
        kRangeDeltaThresh);
  }

  static FloatingPoint Qcdf(FloatingPoint t);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/measurement_model/impl/beam_model_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_
