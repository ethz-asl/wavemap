#ifndef WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_AND_ANGLE_CONTINUOUS_VOLUMETRIC_LOG_ODDS_H_
#define WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_AND_ANGLE_CONTINUOUS_VOLUMETRIC_LOG_ODDS_H_

#include <algorithm>

#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/integrator/measurement_model/approximate_gaussian_distribution.h>
#include <wavemap_common/utils/config_utils.h>
#include <wavemap_common/utils/eigen_format.h>

namespace wavemap {
struct ContinuousVolumetricLogOddsConfig
    : ConfigBase<ContinuousVolumetricLogOddsConfig> {
  FloatingPoint angle_sigma;
  FloatingPoint range_sigma;

  FloatingPoint scaling_free;
  FloatingPoint scaling_occupied;

  bool isValid(bool verbose) const override;
  static ContinuousVolumetricLogOddsConfig from(const param::Map& params);
};

template <int dim>
class ContinuousVolumetricLogOdds {
 public:
  explicit ContinuousVolumetricLogOdds(
      const ContinuousVolumetricLogOddsConfig& config)
      : config_(config.checkValid()) {}

  //  Index<dim> getBottomLeftUpdateIndex() const {
  //    const Point<dim> bottom_left_point = W_start_point_.cwiseMin(
  //        getEndPointOrMaxRange() -
  //        Point<dim>::Constant(max_lateral_component_));
  //    return convert::pointToFloorIndex(bottom_left_point,
  //    min_cell_width_inv_);
  //  }
  //  Index<dim> getTopRightUpdateIndex() const {
  //    const Point<dim> top_right_point = W_start_point_.cwiseMax(
  //        getEndPointOrMaxRange() +
  //        Point<dim>::Constant(max_lateral_component_));
  //    return convert::pointToCeilIndex(top_right_point, min_cell_width_inv_);
  //  }

  const ContinuousVolumetricLogOddsConfig& getConfig() const { return config_; }
  FloatingPoint getAngleThreshold() const { return angle_threshold_; }
  FloatingPoint getRangeDeltaThreshold() const {
    return range_delta_threshold_;
  }

  FloatingPoint computeUpdate(FloatingPoint cell_to_sensor_distance,
                              FloatingPoint cell_to_beam_angle,
                              FloatingPoint measured_distance) const {
    // Compute the full measurement update
    const FloatingPoint g = cell_to_beam_angle / config_.angle_sigma;
    const FloatingPoint f =
        (cell_to_sensor_distance - measured_distance) / config_.range_sigma;
    const FloatingPoint angle_contrib =
        ApproximateGaussianDistribution::cumulative(g + 3.f) -
        ApproximateGaussianDistribution::cumulative(g - 3.f);
    const FloatingPoint range_contrib =
        ApproximateGaussianDistribution::cumulative(f) -
        0.5f * ApproximateGaussianDistribution::cumulative(f - 3.f) - 0.5f;
    const FloatingPoint contribs = angle_contrib * range_contrib;
    const FloatingPoint scaled_contribs =
        ((contribs < 0.f) ? config_.scaling_free * contribs
                          : config_.scaling_occupied * contribs);
    const FloatingPoint p = scaled_contribs + 0.5f;
    const FloatingPoint log_odds = std::log(p / (1.f - p));
    DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
    return log_odds;
  }

 private:
  const ContinuousVolumetricLogOddsConfig config_;

  const FloatingPoint angle_threshold_ = 6.f * config_.angle_sigma;
  const FloatingPoint range_delta_threshold_ = 6.f * config_.range_sigma;
  // NOTE: The angle and upper range thresholds have a width of 6 sigmas because
  //       the assumed 'ground truth' surface thickness is 3 sigma, and the
  //       angular/range uncertainty extends the non-zero regions with another 3
  //       sigma.

  //  // TODO(victorr): Double check this
  //  FloatingPoint max_lateral_component_ =
  //      std::max(std::sin(angle_threshold_) *
  //                   (measured_distance_ + range_delta_threshold_),
  //               range_delta_threshold_);
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_AND_ANGLE_CONTINUOUS_VOLUMETRIC_LOG_ODDS_H_
