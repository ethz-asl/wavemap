#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_

#include <memory>
#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/integrator/projection_model/circular_projector.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"
#include "wavemap_2d/integrator/projective/range_image_1d.h"

namespace wavemap {
class ScanwiseIntegrator2D : public PointcloudIntegrator2D {
 public:
  explicit ScanwiseIntegrator2D(
      const PointcloudIntegratorConfig& config,
      CircularProjector projection_model,
      ContinuousVolumetricLogOdds<2> measurement_model,
      VolumetricDataStructure2D::Ptr occupancy_map)
      : PointcloudIntegrator2D(config, std::move(occupancy_map)),
        measurement_model_(std::move(measurement_model)),
        projection_model_(std::move(projection_model)),
        posed_range_image_(
            std::make_shared<PosedRangeImage1D>(projection_model_)) {
    // TODO(victorr): Check that the pointcloud's angular resolution is lower
    //                than the angular uncertainty of the beam model. This is
    //                necessary since this measurement integrator assumes the
    //                beams don't overlap, i.e. for each sample point we only
    //                evaluate the contribution from the nearest beam.
  }

 protected:
  const ContinuousVolumetricLogOdds<2> measurement_model_;
  const CircularProjector projection_model_;
  std::shared_ptr<PosedRangeImage1D> posed_range_image_;

  FloatingPoint computeUpdate(FloatingPoint d_C_cell,
                              FloatingPoint azimuth_angle_C_cell) {
    if (d_C_cell < config_.min_range || config_.max_range < d_C_cell) {
      return 0.f;
    }

    FloatingPoint angle_remainder;
    const IndexElement idx = projection_model_.angleToNearestIndex(
        azimuth_angle_C_cell, angle_remainder);
    if (idx < 0 || posed_range_image_->getNumBeams() <= idx) {
      return 0.f;
    }
    const FloatingPoint measured_distance = posed_range_image_->operator[](idx);
    if (measured_distance +
            measurement_model_.getRangeThresholdBehindSurface() <
        d_C_cell) {
      return 0.f;
    }

    const FloatingPoint cell_to_beam_angle = std::abs(angle_remainder);
    if (measurement_model_.getAngleThreshold() < cell_to_beam_angle) {
      return 0.f;
    }

    return measurement_model_.computeUpdate(d_C_cell, cell_to_beam_angle,
                                            measured_distance);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_
