#ifndef WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_AND_ANGLE_CONTINUOUS_VOLUMETRIC_LOG_ODDS_H_
#define WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_AND_ANGLE_CONTINUOUS_VOLUMETRIC_LOG_ODDS_H_

#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/integrator/measurement_model/approximate_gaussian_distribution.h>
#include <wavemap_common/integrator/measurement_model/measurement_model_base.h>
#include <wavemap_common/utils/eigen_format.h>

namespace wavemap {
template <int dim>
class ContinuousVolumetricLogOdds : public MeasurementModelBase<dim> {
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
  using MeasurementModelBase<dim>::MeasurementModelBase;

  Index<dim> getBottomLeftUpdateIndex() const override {
    const Point<dim> bottom_left_point = Base::W_start_point_.cwiseMin(
        Base::getEndPointOrMaxRange() -
        Point<dim>::Constant(max_lateral_component_));
    return convert::pointToFloorIndex(bottom_left_point,
                                      Base::min_cell_width_inv_);
  }
  Index<dim> getTopRightUpdateIndex() const override {
    const Point<dim> top_right_point = Base::W_start_point_.cwiseMax(
        Base::getEndPointOrMaxRange() +
        Point<dim>::Constant(max_lateral_component_));
    return convert::pointToCeilIndex(top_right_point,
                                     Base::min_cell_width_inv_);
  }

  // TODO(victorr): Move this method to the integrator as it's 2D / 3D specific
  FloatingPoint computeUpdate(const Index<dim>& index) const {
    const Point<dim> W_cell_center =
        convert::indexToCenterPoint(index, Base::min_cell_width_);
    const Point<dim> W_t_start_point_cell_center =
        W_cell_center - Base::W_start_point_;

    // Compute the distance to the sensor
    const FloatingPoint d_C_cell = W_t_start_point_cell_center.norm();
    // Return early if the point is inside the sensor, beyond the beam's max
    // range, or far behind the surface
    if (d_C_cell < kEpsilon || Base::kRangeMax < d_C_cell ||
        Base::measured_distance_ + kRangeDeltaThresh < d_C_cell) {
      return 0.f;
    }

    // Compute the angle w.r.t. the ray
    const FloatingPoint W_cell_heading_angle = std::atan2(
        W_t_start_point_cell_center.y(), W_t_start_point_cell_center.x());
    const FloatingPoint cell_to_beam_angle =
        std::abs(W_cell_heading_angle - W_beam_heading_angle_);

    // Return early if the point is outside the beam's non-zero angular region
    if (kAngleThresh < cell_to_beam_angle) {
      return 0.f;
    }

    // Compute the full measurement update
    return computeUpdate(d_C_cell, cell_to_beam_angle,
                         Base::measured_distance_);
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
  using Base = MeasurementModelBase<dim>;
  FloatingPoint W_beam_heading_angle_ = 0.f;
  FloatingPoint max_lateral_component_ = 0.f;

  void updateCachedVariablesDerived() override {
    if (Base::measured_distance_ < kEpsilon) {
      W_beam_heading_angle_ = 0.f;
    } else {
      const Point2D W_t_start_end_point =
          Base::W_end_point_ - Base::W_start_point_;
      W_beam_heading_angle_ =
          std::atan2(W_t_start_end_point.y(), W_t_start_end_point.x());
    }
    // TODO(victorr): Calculate this properly
    max_lateral_component_ = std::max(
        std::sin(kAngleThresh) * (Base::measured_distance_ + kRangeDeltaThresh),
        kRangeDeltaThresh);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_AND_ANGLE_CONTINUOUS_VOLUMETRIC_LOG_ODDS_H_
