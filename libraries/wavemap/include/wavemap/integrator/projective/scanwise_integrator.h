#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_H_

#include <memory>
#include <utility>

#include "wavemap/data_structure/image.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h"
#include "wavemap/integrator/pointcloud_integrator.h"
#include "wavemap/integrator/projection_model/image_2d_projection_model.h"

namespace wavemap {
class ScanwiseIntegrator : public PointcloudIntegrator {
 public:
  using Ptr = std::shared_ptr<ScanwiseIntegrator>;
  using BeamOffsetImage = Image<Vector2D>;

  explicit ScanwiseIntegrator(
      const PointcloudIntegratorConfig& config,
      std::shared_ptr<const Image2DProjectionModel> projection_model,
      ContinuousVolumetricLogOdds measurement_model,
      VolumetricDataStructureBase::Ptr occupancy_map)
      : PointcloudIntegrator(config, std::move(occupancy_map)),
        measurement_model_(std::move(measurement_model)),
        projection_model_(std::move(projection_model)),
        posed_range_image_(
            std::make_shared<PosedImage<>>(projection_model_->getDimensions())),
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
  void integrateRangeImage(const PosedImage<>& range_image);

  // Accessors for debugging and visualization
  // NOTE: These accessors are for introspection only, not for modifying the
  //       internal state. They therefore only expose const references or
  //       pointers to const.
  const ContinuousVolumetricLogOdds& getMeasurementModel() const {
    return measurement_model_;
  }
  std::shared_ptr<const Image2DProjectionModel> getProjectionModel() const {
    return projection_model_;
  }
  std::shared_ptr<const PosedImage<>> getPosedRangeImage() const {
    return posed_range_image_;
  }
  const BeamOffsetImage& getBeamOffsetImage() const {
    return beam_offset_image_;
  }

 protected:
  const ContinuousVolumetricLogOdds measurement_model_;
  const std::shared_ptr<const Image2DProjectionModel> projection_model_;

  std::shared_ptr<PosedImage<>> posed_range_image_;
  BeamOffsetImage beam_offset_image_;

  virtual void importPointcloud(const PosedPointcloud<Point3D>& pointcloud);
  virtual void importRangeImage(const PosedImage<>& range_image_input);

  FloatingPoint computeUpdate(const Point3D& C_cell_center) const;

  virtual void updateMap() = 0;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/impl/scanwise_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_SCANWISE_INTEGRATOR_H_
