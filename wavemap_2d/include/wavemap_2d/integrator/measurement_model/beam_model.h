#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/integrator/measurement_model/measurement_model.h"

namespace wavemap_2d {
class BeamModel : public MeasurementModel {
 public:
  static constexpr FloatingPoint kAngleThresh = 0.007853982f;
  static constexpr FloatingPoint kRangeDeltaThresh = 0.1f;
  static constexpr FloatingPoint kAngleSigma = kAngleThresh / 6.f;
  static constexpr FloatingPoint kRangeSigma = kRangeDeltaThresh / 6.f;
  static constexpr FloatingPoint kScaling = 0.5f;
  // NOTE: The angle and upper range thresholds have a width of 6 sigmas because
  //       the ground truth surface thickness is 3 sigma, and the angular/range
  //       uncertainty extends the non-zero regions with another 3 sigma.

  explicit BeamModel(FloatingPoint resolution)
      : MeasurementModel(resolution), max_lateral_component_(0.f) {}

  Index getBottomLeftUpdateIndex() const override;
  Index getTopRightUpdateIndex() const override;

  FloatingPoint computeUpdateAt(const Index& index) const override;
  static FloatingPoint computeUpdate(FloatingPoint cell_to_sensor_distance,
                                     FloatingPoint cell_to_beam_angle,
                                     FloatingPoint measured_distance);

 protected:
  Point C_end_point_;
  Point C_end_point_normalized_;
  FloatingPoint max_lateral_component_;

  void updateCachedVariablesDerived() override {
    C_end_point_ = W_end_point_ - W_start_point_;
    if (measured_distance_ < kEpsilon) {
      C_end_point_normalized_ = Point::Zero();
    } else {
      C_end_point_normalized_ = C_end_point_ / measured_distance_;
    }
    // TODO(victorr): Calculate this properly
    max_lateral_component_ = std::max(
        std::sin(kAngleThresh) * (measured_distance_ + kRangeDeltaThresh),
        kRangeDeltaThresh);
  }

  static FloatingPoint Qcdf(FloatingPoint t);
};
}  // namespace wavemap_2d

#include "wavemap_2d/integrator/measurement_model/impl/beam_model_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_
