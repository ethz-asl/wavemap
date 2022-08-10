#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_

#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/integrator/measurement_model/approximate_gaussian_distribution.h>
#include <wavemap_common/integrator/measurement_model/measurement_model_base.h>
#include <wavemap_common/utils/eigen_format.h>

namespace wavemap {
class BeamModel : public MeasurementModelBase<2> {
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
  using MeasurementModelBase::MeasurementModelBase;

  Index2D getBottomLeftUpdateIndex() const override {
    const Point2D bottom_left_point = W_start_point_.cwiseMin(
        getEndPointOrMaxRange() - Point2D::Constant(max_lateral_component_));
    return convert::pointToFloorIndex(bottom_left_point, min_cell_width_inv_);
  }
  Index2D getTopRightUpdateIndex() const override {
    const Point2D top_right_point = W_start_point_.cwiseMax(
        getEndPointOrMaxRange() + Point2D::Constant(max_lateral_component_));
    return convert::pointToCeilIndex(top_right_point, min_cell_width_inv_);
  }

  FloatingPoint computeUpdateAt(const Index2D& index) const override {
    const Point2D W_cell_center =
        convert::indexToCenterPoint(index, min_cell_width_);
    const Point2D C_cell_center = W_cell_center - W_start_point_;

    // Compute the distance to the sensor
    const FloatingPoint d_C_cell = C_cell_center.norm();
    // Return early if the point is inside the sensor, beyond the beam's max
    // range, or far behind the surface
    if (d_C_cell < kEpsilon || kRangeMax < d_C_cell ||
        measured_distance_ + kRangeDeltaThresh < d_C_cell) {
      return 0.f;
    }

    // Compute the angle w.r.t. the ray
    // NOTE: For a method that works in 3D as well as 2D, see commit
    //       4abe1af9fc66cc069697727cc14577ade3abe092 or earlier.
    const FloatingPoint cell_angle =
        std::atan2(C_cell_center.y(), C_cell_center.x());
    const FloatingPoint cell_to_beam_angle = std::abs(cell_angle - beam_angle_);

    // Return early if the point is outside the beam's non-zero angular region
    if (kAngleThresh < cell_to_beam_angle) {
      return 0.f;
    }

    // Compute the full measurement update
    return computeUpdate(d_C_cell, cell_to_beam_angle, measured_distance_);
  }
  static FloatingPoint computeUpdate(FloatingPoint cell_to_sensor_distance,
                                     FloatingPoint cell_to_beam_angle,
                                     FloatingPoint measured_distance) {
    // Compute the full measurement update
    const FloatingPoint g = cell_to_beam_angle / kAngleSigma;
    const FloatingPoint f =
        (cell_to_sensor_distance - measured_distance) / kRangeSigma;
    const FloatingPoint angle_contrib =
        ApproximateGaussianDistribution::cumulative(g + 3.f) -
        ApproximateGaussianDistribution::cumulative(g - 3.f);
    const FloatingPoint range_contrib =
        ApproximateGaussianDistribution::cumulative(f) -
        0.5f * ApproximateGaussianDistribution::cumulative(f - 3.f) - 0.5f;
    const FloatingPoint scaled_contribs =
        kScaling * range_contrib * angle_contrib;
    const FloatingPoint p = scaled_contribs + 0.5f;
    const FloatingPoint log_odds = std::log(p / (1.f - p));
    DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
    return log_odds;
  }

 private:
  FloatingPoint beam_angle_;
  FloatingPoint max_lateral_component_ = 0.f;

  void updateCachedVariablesDerived() override {
    if (measured_distance_ < kEpsilon) {
      beam_angle_ = 0.f;
    } else {
      const Point2D C_end_point = W_end_point_ - W_start_point_;
      beam_angle_ = std::atan2(C_end_point.y(), C_end_point.x());
    }
    // TODO(victorr): Calculate this properly
    max_lateral_component_ = std::max(
        std::sin(kAngleThresh) * (measured_distance_ + kRangeDeltaThresh),
        kRangeDeltaThresh);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BEAM_MODEL_H_
