#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_

#include <memory>
#include <utility>

#include <wavemap/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap/integrator/projection_model/image_2d/image_2d_projection_model.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"
#include "wavemap_3d/integrator/projective/beam_offset_image_2d.h"
#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
class ScanwiseIntegrator3D : public PointcloudIntegrator3D {
 public:
  using Ptr = std::shared_ptr<ScanwiseIntegrator3D>;

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
        beam_offset_image_(projection_model_->getDimensions()) {
    // TODO(victorr): Check that the pointcloud's angular resolution is lower
    //                than the angular uncertainty of the beam model. This is
    //                necessary since this measurement integrator assumes the
    //                beams don't overlap, i.e. for each sample point we only
    //                evaluate the contribution from the nearest beam.
  }

  // Methods to integrate new pointclouds / depth images into the map
  void integratePointcloud(
      const PosedPointcloud<Point<3>>& pointcloud) override;
  void integrateRangeImage(const PosedRangeImage2D& range_image);

  // Accessors for debugging and visualization
  // NOTE: These accessors are for introspection only, not for modifying the
  //       internal state. They therefore only expose const references or
  //       pointers to const.
  const ContinuousVolumetricLogOdds<3>& getMeasurementModel() const {
    return measurement_model_;
  }
  std::shared_ptr<const Image2DProjectionModel> getProjectionModel() const {
    return projection_model_;
  }
  std::shared_ptr<const PosedRangeImage2D> getPosedRangeImage() const {
    return posed_range_image_;
  }
  const BeamOffsetImage2D& getBeamOffsetImage() const {
    return beam_offset_image_;
  }

 protected:
  const ContinuousVolumetricLogOdds<3> measurement_model_;
  const std::shared_ptr<const Image2DProjectionModel> projection_model_;

  std::shared_ptr<PosedRangeImage2D> posed_range_image_;
  BeamOffsetImage2D beam_offset_image_;

  virtual void importPointcloud(const PosedPointcloud<Point3D>& pointcloud);
  virtual void importRangeImage(const PosedRangeImage2D& range_image_input);

  FloatingPoint computeUpdate(const Point3D& C_cell_center) const;

  virtual void updateMap() = 0;
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/impl/scanwise_integrator_3d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_3D_H_
