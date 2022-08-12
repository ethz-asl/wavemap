#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_

#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/integrator/projection_model/spherical_projector.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"
#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
constexpr FloatingPoint kOusterVerticalFoV = 0.3926991f;
class ScanwiseIntegrator3D : public PointcloudIntegrator3D {
 public:
  explicit ScanwiseIntegrator3D(VolumetricDataStructure3D::Ptr occupancy_map)
      : PointcloudIntegrator3D(std::move(occupancy_map)),
        spherical_projector_(-kOusterVerticalFoV, kOusterVerticalFoV, 64, -kPi,
                             kPi, 1024) {
    // TODO(victorr): Make the FoV and number of beams configurable
    // TODO(victorr): Check that the pointcloud's angular resolution is lower
    //                than the angular uncertainty of the beam model. This is
    //                necessary since this measurement integrator assumes the
    //                beams don't overlap, i.e. for each sample point we only
    //                evaluate the contribution from the nearest beam.
  }

 protected:
  SphericalProjector spherical_projector_;

  FloatingPoint computeUpdate(const RangeImage2D& range_image,
                              FloatingPoint d_C_cell,
                              const Vector2D& spherical_C_cell) {
    if (d_C_cell < kEpsilon ||
        ContinuousVolumetricLogOdds<3>::kRangeMax < d_C_cell) {
      return 0.f;
    }

    Vector2D spherical_remainders;
    const Index2D idx = spherical_projector_.sphericalToNearestIndex(
        spherical_C_cell, spherical_remainders);
    if ((idx.array() < 0 || range_image.getDimensions().array() <= idx.array())
            .any()) {
      return 0.f;
    }
    const FloatingPoint measured_distance = range_image[idx];
    if (measured_distance + ContinuousVolumetricLogOdds<3>::kRangeDeltaThresh <
        d_C_cell) {
      return 0.f;
    }

    // TODO(victorr): Test if this approximation is valid (considering the angle
    //                is always small)
    const FloatingPoint cell_to_beam_angle = spherical_remainders.norm();
    if (ContinuousVolumetricLogOdds<3>::kAngleThresh < cell_to_beam_angle) {
      return 0.f;
    }

    return ContinuousVolumetricLogOdds<3>::computeUpdate(
        d_C_cell, cell_to_beam_angle, measured_distance);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
