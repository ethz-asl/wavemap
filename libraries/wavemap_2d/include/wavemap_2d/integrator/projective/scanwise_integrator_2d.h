#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_

#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/integrator/projection_model/circular_projector.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"
#include "wavemap_2d/integrator/projective/range_image_1d.h"

namespace wavemap {
class ScanwiseIntegrator2D : public PointcloudIntegrator2D {
 public:
  explicit ScanwiseIntegrator2D(VolumetricDataStructure2D::Ptr occupancy_map)
      : PointcloudIntegrator2D(std::move(occupancy_map)),
        circular_projector_(-kHalfPi, kHalfPi, 400) {
    // TODO(victorr): Make the FoV and number of beams configurable
    // TODO(victorr): Check that the pointcloud's angular resolution is lower
    //                than the angular uncertainty of the beam model. This is
    //                necessary since this measurement integrator assumes the
    //                beams don't overlap, i.e. for each sample point we only
    //                evaluate the contribution from the nearest beam.
  }

 protected:
  CircularProjector circular_projector_;

  FloatingPoint computeUpdate(const RangeImage1D& range_image,
                              FloatingPoint d_C_cell,
                              FloatingPoint azimuth_angle_C_cell) {
    if (d_C_cell < kEpsilon ||
        ContinuousVolumetricLogOdds<2>::kRangeMax < d_C_cell) {
      return 0.f;
    }

    const IndexElement idx =
        circular_projector_.angleToNearestIndex(azimuth_angle_C_cell);
    if (idx < 0 || range_image.getNumBeams() <= idx) {
      return 0.f;
    }
    const FloatingPoint measured_distance = range_image[idx];
    if (measured_distance + ContinuousVolumetricLogOdds<2>::kRangeDeltaThresh <
        d_C_cell) {
      return 0.f;
    }

    const FloatingPoint beam_azimuth_angle =
        circular_projector_.indexToAngle(idx);
    const FloatingPoint cell_to_beam_angle =
        std::abs(azimuth_angle_C_cell - beam_azimuth_angle);
    if (ContinuousVolumetricLogOdds<2>::kAngleThresh < cell_to_beam_angle) {
      return 0.f;
    }

    return ContinuousVolumetricLogOdds<2>::computeUpdate(
        d_C_cell, cell_to_beam_angle, measured_distance);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_
