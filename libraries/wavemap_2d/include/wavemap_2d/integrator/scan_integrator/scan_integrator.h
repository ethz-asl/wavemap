#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_

#include <utility>

#include <wavemap_common/integrator/circle_projector.h>

#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap {
class ScanIntegrator : public PointcloudIntegrator {
 public:
  explicit ScanIntegrator(VolumetricDataStructure2D::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)),
        circle_projector_(-kHalfPi, kHalfPi, 400) {
    // TODO(victorr): Make the FoV and number of beams configurable
    // TODO(victorr): Check that the pointcloud's angular resolution is lower
    //                than the angular uncertainty of the beam model. This is
    //                necessary since this measurement integrator assumes the
    //                beams don't overlap, i.e. for each sample point we only
    //                evaluate the contribution from the nearest beam.
  }

 protected:
  CircleProjector circle_projector_;

  static FloatingPoint sampleUpdateAtPoint(
      const RangeImage& range_image, const CircleProjector& circle_projector,
      FloatingPoint d_C_cell, FloatingPoint azimuth_angle_C_cell);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/scan_integrator/impl/scan_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_
