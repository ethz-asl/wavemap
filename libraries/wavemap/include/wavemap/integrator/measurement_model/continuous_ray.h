#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_RAY_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_RAY_H_

#include <memory>
#include <utility>

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/integrator/measurement_model/measurement_model_base.h"
#include "wavemap/integrator/projective/update_type.h"

namespace wavemap {
struct ContinuousRayConfig
    : ConfigBase<ContinuousRayConfig, 4, BeamSelectorType> {
  FloatingPoint range_sigma = 0.f;
  FloatingPoint scaling_free = 0.5f;
  FloatingPoint scaling_occupied = 0.5f;

  BeamSelectorType beam_selector_type = BeamSelectorType::kNearestNeighbor;

  static MemberMap memberMap;

  // Constructors
  ContinuousRayConfig() = default;
  ContinuousRayConfig(FloatingPoint range_sigma, FloatingPoint scaling_free,
                      FloatingPoint scaling_occupied)
      : range_sigma(range_sigma),
        scaling_free(scaling_free),
        scaling_occupied(scaling_occupied) {}

  bool isValid(bool verbose) const override;
};

class ContinuousRay : public MeasurementModelBase {
 public:
  explicit ContinuousRay(const ContinuousRayConfig& config,
                         ProjectorBase::ConstPtr projection_model,
                         Image<>::ConstPtr range_image)
      : config_(config.checkValid()),
        projection_model_(std::move(projection_model)),
        range_image_(std::move(range_image)) {}

  const ContinuousRayConfig& getConfig() const { return config_; }
  FloatingPoint getPaddingAngle() const override { return 0.f; }
  FloatingPoint getPaddingSurfaceFront() const override {
    return range_threshold_front_;
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
  const ContinuousRayConfig config_;

  const ProjectorBase::ConstPtr projection_model_;
  const Image<>::ConstPtr range_image_;

  const FloatingPoint range_threshold_front_ = 3.f * config_.range_sigma;
  const FloatingPoint range_threshold_back_ = 6.f * config_.range_sigma;
  // NOTE: The upper range thresholds has a width of 6 sigmas because the
  //       assumed 'ground truth' surface thickness is 3 sigma, and the range
  //       uncertainty extends the non-zero regions with another 3 sigma.

  FloatingPoint computeBeamUpdate(const Vector3D& sensor_coordinates,
                                  const Index2D& image_index) const;

  // Compute the full measurement update, i.e. valid anywhere
  FloatingPoint computeFullBeamUpdate(FloatingPoint cell_to_sensor_distance,
                                      FloatingPoint measured_distance) const;

  // Compute the measurement update given that we're fully in free space, i.e.
  // only in the interval [ 0, measured_distance - range_threshold_in_front [
  // NOTE: Using this method is optional. It's slightly cheaper and more
  // accurate, but the difference is minor.
  FloatingPoint computeFreeSpaceBeamUpdate() const;
};
}  // namespace wavemap

#include "wavemap/integrator/measurement_model/impl/continuous_ray_inl.h"

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONTINUOUS_RAY_H_
