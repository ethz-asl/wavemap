#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_

#include <memory>
#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/integrator/projection_model/image_2d/ouster_projector.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"
#include "wavemap_3d/integrator/projective/beam_offset_image_2d.h"
#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
class ScanwiseIntegrator3D : public PointcloudIntegrator3D {
 public:
  explicit ScanwiseIntegrator3D(
      const PointcloudIntegratorConfig& config,
      std::shared_ptr<const Image2DProjectionModel> projection_model,
      ContinuousVolumetricLogOdds<3> measurement_model,
      VolumetricDataStructure3D::Ptr occupancy_map)
      : PointcloudIntegrator3D(config, std::move(occupancy_map)),
        measurement_model_(std::move(measurement_model)),
        projection_model_(std::move(projection_model)),
        posed_range_image_(std::make_shared<PosedRangeImage2D>(
            projection_model_->getDimensions())),
        bearing_image_(projection_model_->getDimensions()) {
    // TODO(victorr): Check that the pointcloud's angular resolution is lower
    //                than the angular uncertainty of the beam model. This is
    //                necessary since this measurement integrator assumes the
    //                beams don't overlap, i.e. for each sample point we only
    //                evaluate the contribution from the nearest beam.
  }

 protected:
  const ContinuousVolumetricLogOdds<3> measurement_model_;
  const std::shared_ptr<const Image2DProjectionModel> projection_model_;
  std::shared_ptr<PosedRangeImage2D> posed_range_image_;
  BeamOffsetImage2D bearing_image_;

  void updateRangeImage(const PosedPointcloud<Point3D>& pointcloud,
                        PosedRangeImage2D& posed_range_image,
                        BeamOffsetImage2D& bearing_image) const;

  FloatingPoint computeUpdate(const Point3D& C_cell_center) const;
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/impl/scanwise_integrator_3d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
