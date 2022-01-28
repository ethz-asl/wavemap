#ifndef WAVEMAP_2D_INTEGRATOR_BEAM_MODEL_H_
#define WAVEMAP_2D_INTEGRATOR_BEAM_MODEL_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/integrator/grid_iterator.h"
#include "wavemap_2d/integrator/measurement_model_base.h"

namespace wavemap_2d {
class BeamModel : public MeasurementModelBase {
 public:
  static constexpr FloatingPoint kAngleThresh = 0.007853982f;
  static constexpr FloatingPoint kRangeDeltaThresh = 0.1f;
  static constexpr FloatingPoint kAngleSigma = kAngleThresh / 3.f;
  static constexpr FloatingPoint kRangeSigma = kRangeDeltaThresh / 3.f;
  static constexpr FloatingPoint kFreeScaling = 0.2f;
  static constexpr FloatingPoint kOccScaling = 0.4f;

  explicit BeamModel(FloatingPoint resolution)
      : MeasurementModelBase(resolution), max_lateral_component_(0.f) {}

  Index getBottomLeftUpdateIndex() const override;
  Index getTopRightUpdateIndex() const override;

  FloatingPoint computeUpdateAt(const Index& index) const override;
  void updateMap(DataStructureBase& map) const override;

 protected:
  Point C_end_point_;
  Point C_end_point_normalized_;
  FloatingPoint max_lateral_component_;
  // TODO(victorr): Consider calculating the true min/max lateral components,
  //                or using a better approximation
  Index getUpdateMinIndex() const;
  Index getUpdateMaxIndex() const;

  void updateCachedVariablesDerived() override {
    C_end_point_ = W_end_point_ - W_start_point_;
    C_end_point_normalized_ = C_end_point_ / measured_distance_;
    max_lateral_component_ =
        std::sin(kAngleThresh) * (measured_distance_ + kRangeDeltaThresh);
  }

  static FloatingPoint Qcdf(FloatingPoint t);
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_BEAM_MODEL_H_
