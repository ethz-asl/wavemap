#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_BEAM_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_BEAM_H_

#include <algorithm>
#include <memory>
#include <utility>

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/integrator/measurement_model/measurement_model_base.h"
#include "wavemap/integrator/projective/update_type.h"
#include "wavemap/utils/eigen_format.h"

namespace wavemap {
struct ContinuousBeamConfig : ConfigBase<ContinuousBeamConfig> {
  FloatingPoint angle_sigma = 0.f;
  FloatingPoint range_sigma = 0.f;
  FloatingPoint scaling_free = 0.5f;
  FloatingPoint scaling_occupied = 0.5f;

  BeamSelectorType beam_selector_type = BeamSelectorType::kAllNeighbors;

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
  static ContinuousBeamConfig from(
      const param::Map& params,
      SiUnit image_coordinates_unit = SiUnit::kRadians);
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
        beam_offset_image_(std::move(beam_offset_image)) {}

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
      const Vector3D& sensor_coordinates) const override;

 private:
  const ContinuousBeamConfig config_;

  const ProjectorBase::ConstPtr projection_model_;
  const Image<>::ConstPtr range_image_;
  const Image<Vector2D>::ConstPtr beam_offset_image_;

  const FloatingPoint angle_threshold_ = 6.f * config_.angle_sigma;
  const FloatingPoint range_threshold_front = 3.f * config_.range_sigma;
  const FloatingPoint range_threshold_back_ = 6.f * config_.range_sigma;
  // NOTE: The angle and upper range thresholds have a width of 6 sigmas because
  //       the assumed 'ground truth' surface thickness is 3 sigma, and the
  //       angular/range uncertainty extends the non-zero regions with another 3
  //       sigma.

  // Compute the measurement update for a single beam
  FloatingPoint computeBeamUpdate(FloatingPoint cell_to_sensor_distance,
                                  FloatingPoint cell_to_beam_image_error_norm,
                                  FloatingPoint measured_distance) const;
};
}  // namespace wavemap

#include "wavemap/integrator/measurement_model/impl/continuous_beam_inl.h"

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_BEAM_H_
