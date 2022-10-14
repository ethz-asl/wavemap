#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_

#include <memory>
#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/integrator/projection_model/spherical_projector.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"
#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
class ScanwiseIntegrator3D : public PointcloudIntegrator3D {
 public:
  explicit ScanwiseIntegrator3D(
      const PointcloudIntegratorConfig& config,
      SphericalProjector projection_model,
      ContinuousVolumetricLogOdds<3> measurement_model,
      VolumetricDataStructure3D::Ptr occupancy_map)
      : PointcloudIntegrator3D(config, std::move(occupancy_map)),
        measurement_model_(std::move(measurement_model)),
        projection_model_(std::move(projection_model)),
        posed_range_image_(
            std::make_shared<PosedRangeImage2D>(projection_model_)) {
    // TODO(victorr): Check that the pointcloud's angular resolution is lower
    //                than the angular uncertainty of the beam model. This is
    //                necessary since this measurement integrator assumes the
    //                beams don't overlap, i.e. for each sample point we only
    //                evaluate the contribution from the nearest beam.
  }

 protected:
  const ContinuousVolumetricLogOdds<3> measurement_model_;
  const SphericalProjector projection_model_;
  std::shared_ptr<PosedRangeImage2D> posed_range_image_;

  FloatingPoint computeUpdate(FloatingPoint d_C_cell,
                              const Vector2D& spherical_C_cell) {
    if (d_C_cell < kEpsilon || config_.max_range < d_C_cell) {
      return 0.f;
    }

    Vector2D spherical_remainders;
    const Index2D idx = projection_model_.sphericalToNearestIndex(
        spherical_C_cell, spherical_remainders);
    if ((idx.array() < 0).any() ||
        (posed_range_image_->getDimensions().array() <= idx.array()).any()) {
      return 0.f;
    }
    const FloatingPoint measured_distance = posed_range_image_->operator[](idx);
    if (measured_distance +
            measurement_model_.getRangeThresholdBehindSurface() <
        d_C_cell) {
      return 0.f;
    }

    // TODO(victorr): Test if this approximation is valid (considering the angle
    //                is always small)
    const FloatingPoint cell_to_beam_angle = spherical_remainders.norm();
    if (measurement_model_.getAngleThreshold() < cell_to_beam_angle) {
      return 0.f;
    }

    return measurement_model_.computeUpdate(d_C_cell, cell_to_beam_angle,
                                            measured_distance);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
