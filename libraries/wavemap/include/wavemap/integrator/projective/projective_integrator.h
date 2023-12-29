#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_PROJECTIVE_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_PROJECTIVE_INTEGRATOR_H_

#include <memory>
#include <utility>

#include "wavemap/config/type_selector.h"
#include "wavemap/data_structure/image.h"
#include "wavemap/integrator/integrator_base.h"
#include "wavemap/integrator/measurement_model/measurement_model_base.h"
#include "wavemap/integrator/projection_model/projector_base.h"
#include "wavemap/map/map_base.h"

namespace wavemap {
/**
 * Config struct for projective integrators.
 */
struct ProjectiveIntegratorConfig : ConfigBase<ProjectiveIntegratorConfig, 4> {
  //! Minimum range measurements should have to be considered.
  //! Measurements below this threshold are ignored.
  Meters<FloatingPoint> min_range = 0.5f;
  //! Maximum range up to which to update the map.
  //! Measurements that exceed this range are used as free-space beams,
  //! up to the maximum range.
  Meters<FloatingPoint> max_range = 20.f;

  //! Maximum resolution to use for this integrator, set as the height at which
  //! to terminate the coarse-to-fine measurement updates. Defaults to 0 (max
  //! res). Can be set to 1 for 1/2 of the max resolution, to 2 for 1/4 of the
  //! max resolution, etc.This can be used to fuse multiple inputs with
  //! different maximum resolutions into a single map.
  IndexElement termination_height = 0;
  //! The update error threshold at which the coarse-to-fine measurement
  //! integrator is allowed to terminate, in log-odds. For more information,
  //! please refer to: https://www.roboticsproceedings.org/rss19/p065.pdf.
  FloatingPoint termination_update_error = 0.1f;

  static MemberMap memberMap;

  // Constructors
  ProjectiveIntegratorConfig() = default;
  ProjectiveIntegratorConfig(FloatingPoint min_range, FloatingPoint max_range)
      : min_range(min_range), max_range(max_range) {}

  bool isValid(bool verbose) const override;
};

class ProjectiveIntegrator : public IntegratorBase {
 public:
  using Ptr = std::shared_ptr<ProjectiveIntegrator>;

  explicit ProjectiveIntegrator(
      const ProjectiveIntegratorConfig& config,
      ProjectorBase::ConstPtr projection_model,
      PosedImage<>::Ptr posed_range_image,
      Image<Vector2D>::Ptr beam_offset_image,
      MeasurementModelBase::ConstPtr measurement_model)
      : config_(config.checkValid()),
        projection_model_(std::move(CHECK_NOTNULL(projection_model))),
        posed_range_image_(std::move(CHECK_NOTNULL(posed_range_image))),
        beam_offset_image_(std::move(CHECK_NOTNULL(beam_offset_image))),
        measurement_model_(std::move(CHECK_NOTNULL(measurement_model))) {}

  // Methods to integrate new pointclouds / depth images into the map
  void integratePointcloud(
      const PosedPointcloud<Point<3>>& pointcloud) override;
  void integrateRangeImage(const PosedImage<>& range_image);

  // Accessors for debugging and visualization
  // NOTE: These accessors are for introspection only, not for modifying the
  //       internal state. They therefore only expose const references or
  //       pointers to const.
  const MeasurementModelBase::ConstPtr& getMeasurementModel() const {
    return measurement_model_;
  }
  ProjectorBase::ConstPtr getProjectionModel() const {
    return projection_model_;
  }
  PosedImage<>::ConstPtr getPosedRangeImage() const {
    return posed_range_image_;
  }
  Image<Vector2D>::ConstPtr getBeamOffsetImage() const {
    return beam_offset_image_;
  }

 protected:
  const ProjectiveIntegratorConfig config_;

  const ProjectorBase::ConstPtr projection_model_;
  const PosedImage<>::Ptr posed_range_image_;
  const Image<Vector2D>::Ptr beam_offset_image_;

  const MeasurementModelBase::ConstPtr measurement_model_;

  virtual void importPointcloud(const PosedPointcloud<>& pointcloud);
  virtual void importRangeImage(const PosedImage<>& range_image_input);

  virtual void updateMap() = 0;

  FloatingPoint computeUpdate(const Point3D& C_cell_center) const;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/impl/projective_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_PROJECTIVE_INTEGRATOR_H_
