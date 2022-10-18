#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_

#include <memory>
#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/integrator/projection_model/image_1d/circular_projector.h>

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
        posed_range_image_(std::make_shared<PosedRangeImage1D>(
            projection_model_.getNumCells())) {
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

  void updateRangeImage(const PosedPointcloud<Point2D>& pointcloud,
                        PosedRangeImage1D& posed_range_image) const;

  FloatingPoint computeUpdate(const Point2D& C_cell_center) const;
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/impl/scanwise_integrator_2d_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_2D_H_
