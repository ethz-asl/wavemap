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
  FloatingPoint angle_sigma = 0.f;
  FloatingPoint range_sigma = 0.f;
  FloatingPoint scaling_free = 0.5f;
  FloatingPoint scaling_occupied = 0.5f;

  // Constructors
  ContinuousVolumetricLogOddsConfig() = default;
  ContinuousVolumetricLogOddsConfig(FloatingPoint angle_sigma,
                                    FloatingPoint range_sigma,
                                    FloatingPoint scaling_free,
                                    FloatingPoint scaling_occupied)
      : angle_sigma(angle_sigma),
        range_sigma(range_sigma),
        scaling_free(scaling_free),
        scaling_occupied(scaling_occupied) {}

  bool isValid(bool verbose) const override;
  static ContinuousVolumetricLogOddsConfig from(const param::Map& params);
};

template <int dim>
class ContinuousVolumetricLogOdds {
 public:
  explicit ContinuousVolumetricLogOdds(
      const ContinuousVolumetricLogOddsConfig& config)
      : config_(config.checkValid()) {}

  Index<dim> getBottomLeftUpdateIndex(const Point<dim>& W_start_point,
                                      const Point<dim>& W_end_point,
                                      FloatingPoint measured_distance,
                                      FloatingPoint min_cell_width_inv) const {
    const Point<dim> bottom_left_point =
        W_start_point.cwiseMin(W_end_point) -
        Point<dim>::Constant(getCombinedThreshold(measured_distance));
    return convert::pointToFloorIndex(bottom_left_point, min_cell_width_inv);
  }
  Index<dim> getTopRightUpdateIndex(const Point<dim>& W_start_point,
                                    const Point<dim>& W_end_point,
                                    FloatingPoint measured_distance,
                                    FloatingPoint min_cell_width_inv) const {
    const Point<dim> top_right_point =
        W_start_point.cwiseMax(W_end_point) +
        Point<dim>::Constant(getCombinedThreshold(measured_distance));
    return convert::pointToCeilIndex(top_right_point, min_cell_width_inv);
  }

  const ContinuousVolumetricLogOddsConfig& getConfig() const { return config_; }
  FloatingPoint getAngleThreshold() const { return angle_threshold_; }
  FloatingPoint getRangeThresholdInFrontOfSurface() const {
    return range_threshold_in_front;
  }
  FloatingPoint getRangeThresholdBehindSurface() const {
    return range_threshold_behind_;
  }
  FloatingPoint getCombinedThreshold(FloatingPoint measured_distance) const {
    const FloatingPoint max_lateral_component =
        std::max(std::sin(angle_threshold_) *
                     (measured_distance + range_threshold_behind_),
                 range_threshold_behind_);
    return max_lateral_component;
  }

  // Compute the full measurement update, i.e. valid anywhere
  FloatingPoint computeUpdate(FloatingPoint cell_to_sensor_distance,
                              FloatingPoint cell_to_beam_angle,
                              FloatingPoint measured_distance) const {
    const FloatingPoint f =
        (cell_to_sensor_distance - measured_distance) / config_.range_sigma;
    const FloatingPoint range_contrib =
        ApproximateGaussianDistribution::cumulative(f) -
        0.5f * ApproximateGaussianDistribution::cumulative(f - 3.f) - 0.5f;

    const FloatingPoint g = cell_to_beam_angle / config_.angle_sigma;
    const FloatingPoint angle_contrib =
        ApproximateGaussianDistribution::cumulative(g + 3.f) -
        ApproximateGaussianDistribution::cumulative(g - 3.f);

    const FloatingPoint contribs = range_contrib * angle_contrib;
    const FloatingPoint scaled_contribs =
        (contribs < 0.f) ? config_.scaling_free * contribs
                         : config_.scaling_occupied * contribs;

    const FloatingPoint p = scaled_contribs + 0.5f;
    const FloatingPoint log_odds = std::log(p / (1.f - p));
    DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
    return log_odds;
  }

  // Compute the measurement update given that we're fully in free space, i.e.
  // only in the interval [ 0, measured_distance - range_threshold_in_front [
  // NOTE: Using this method is optional. It's slightly cheaper and more
  // accurate, but the difference is minor.
  FloatingPoint computeFreeSpaceUpdate(FloatingPoint cell_to_beam_angle) const {
    constexpr FloatingPoint kFreeSpaceRangeContrib = -0.5f;

    const FloatingPoint g = cell_to_beam_angle / config_.angle_sigma;
    const FloatingPoint angle_contrib =
        ApproximateGaussianDistribution::cumulative(g + 3.f) -
        ApproximateGaussianDistribution::cumulative(g - 3.f);

    const FloatingPoint contribs = kFreeSpaceRangeContrib * angle_contrib;
    const FloatingPoint scaled_contribs = config_.scaling_free * contribs;

    const FloatingPoint p = scaled_contribs + 0.5f;
    const FloatingPoint log_odds = std::log(p / (1.f - p));
    DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
    return log_odds;
  }

 private:
  const ContinuousVolumetricLogOddsConfig config_;

  const FloatingPoint angle_threshold_ = 6.f * config_.angle_sigma;
  const FloatingPoint range_threshold_in_front = 3.f * config_.range_sigma;
  const FloatingPoint range_threshold_behind_ = 6.f * config_.range_sigma;
  // NOTE: The angle and upper range thresholds have a width of 6 sigmas because
  //       the assumed 'ground truth' surface thickness is 3 sigma, and the
  //       angular/range uncertainty extends the non-zero regions with another 3
  //       sigma.
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_AND_ANGLE_CONTINUOUS_VOLUMETRIC_LOG_ODDS_H_
