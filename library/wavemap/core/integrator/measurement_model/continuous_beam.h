#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_BEAM_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_BEAM_H_

#include <algorithm>
#include <memory>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/config/config_base.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/integrator/measurement_model/measurement_model_base.h"
#include "wavemap/core/integrator/projective/update_type.h"
#include "wavemap/core/utils/print/eigen.h"

namespace wavemap {
/**
 * Config struct for the continuous beam measurement model.
 */
struct ContinuousBeamConfig
    : ConfigBase<ContinuousBeamConfig, 5, BeamSelectorType> {
  //! Uncertainty along the angle axis.
  Radians<FloatingPoint> angle_sigma = 0.f;
  //! Uncertainty along the range axis.
  Meters<FloatingPoint> range_sigma = 0.f;
  //! Scale factor to apply to the continuous Bayesian occupancy model for cells
  //! that are observed as free. This can, for example, be used to give a higher
  //! weight to occupied updates than free updates.
  FloatingPoint scaling_free = 0.5f;
  //! Scale factor to apply to the continuous Bayesian occupancy model for cells
  //! that are observed as occupied. This can, for example, be used to give a
  //! higher weight to occupied updates than free updates.
  FloatingPoint scaling_occupied = 0.5f;

  //! Which neighboring beams to consider when computing a cell's measurement
  //! update.
  BeamSelectorType beam_selector_type = BeamSelectorType::kAllNeighbors;

  static MemberMap memberMap;

  // Constructors
  ContinuousBeamConfig() = default;
  ContinuousBeamConfig(FloatingPoint angle_sigma, FloatingPoint range_sigma,
                       FloatingPoint scaling_free,
                       FloatingPoint scaling_occupied)
      : angle_sigma(angle_sigma),
        range_sigma(range_sigma),
        scaling_free(scaling_free),
        scaling_occupied(scaling_occupied) {}

  bool isValid(bool verbose) const override;
};

class ContinuousBeam : public MeasurementModelBase {
 public:
  explicit ContinuousBeam(const ContinuousBeamConfig& config,
                          ProjectorBase::ConstPtr projection_model,
                          Image<>::ConstPtr range_image,
                          Image<Vector2D>::ConstPtr beam_offset_image)
      : config_(config.checkValid()),
        projection_model_(std::move(projection_model)),
        range_image_(std::move(range_image)),
        beam_offset_image_(std::move(beam_offset_image)) {
    CHECK_EQ(range_image_->getDimensions(),
             beam_offset_image_->getDimensions());
  }

  const ContinuousBeamConfig& getConfig() const { return config_; }
  FloatingPoint getPaddingAngle() const override { return angle_threshold_; }
  FloatingPoint getPaddingSurfaceFront() const override {
    return range_threshold_front;
  }
  FloatingPoint getPaddingSurfaceBack() const override {
    return range_threshold_back_;
  }

  FloatingPoint computeWorstCaseApproximationError(
      UpdateType update_type, FloatingPoint cell_to_sensor_distance,
      FloatingPoint cell_bounding_radius) const override;

  FloatingPoint computeUpdate(
      const SensorCoordinates& sensor_coordinates) const override;

 private:
  const ContinuousBeamConfig config_;

  const ProjectorBase::ConstPtr projection_model_;
  const Image<>::ConstPtr range_image_;
  const Image<Vector2D>::ConstPtr beam_offset_image_;

  const FloatingPoint angle_threshold_ = 6.f * config_.angle_sigma;
  const FloatingPoint angle_threshold_squared =
      angle_threshold_ * angle_threshold_;
  const FloatingPoint range_threshold_front = 3.f * config_.range_sigma;
  const FloatingPoint range_threshold_back_ = 6.f * config_.range_sigma;
  // NOTE: The angle and upper range thresholds have a width of 6 sigmas because
  //       the assumed 'ground truth' surface thickness is 3 sigma, and the
  //       angular/range uncertainty extends the non-zero regions with another 3
  //       sigma.

  // Compute the measurement update for a neighborhood in the range image
  FloatingPoint computeBeamUpdateNearestNeighbor(
      const Image<>& range_image, const Image<Vector2D>& beam_offset_image,
      const ProjectorBase& projection_model,
      const SensorCoordinates& sensor_coordinates) const;
  FloatingPoint computeBeamUpdateAllNeighbors(
      const Image<>& range_image, const Image<Vector2D>& beam_offset_image,
      const ProjectorBase& projection_model,
      const SensorCoordinates& sensor_coordinates) const;

  // Compute the measurement update for a single beam
  FloatingPoint computeBeamUpdate(
      FloatingPoint cell_to_sensor_distance,
      FloatingPoint cell_to_beam_image_error_norm_squared,
      FloatingPoint measured_distance) const;
};
}  // namespace wavemap

#include "wavemap/core/integrator/measurement_model/impl/continuous_beam_inl.h"

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_BEAM_H_
