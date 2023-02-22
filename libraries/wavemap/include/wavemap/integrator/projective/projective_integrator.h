#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_PROJECTIVE_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_PROJECTIVE_INTEGRATOR_H_

#include <memory>
#include <utility>

#include "wavemap/config/type_selector.h"
#include "wavemap/data_structure/image.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/integrator_base.h"
#include "wavemap/integrator/measurement_model/measurement_model_base.h"
#include "wavemap/integrator/projection_model/projector_base.h"

namespace wavemap {
struct ProjectiveIntegratorConfig : ConfigBase<ProjectiveIntegratorConfig> {
  FloatingPoint min_range = 0.5f;
  FloatingPoint max_range = 20.f;

  NdtreeIndexElement termination_height = 0;
  FloatingPoint termination_update_error = 0.1f;

  // Constructors
  ProjectiveIntegratorConfig() = default;
  ProjectiveIntegratorConfig(FloatingPoint min_range, FloatingPoint max_range)
      : min_range(min_range), max_range(max_range) {}

  bool isValid(bool verbose) const override;
  static ProjectiveIntegratorConfig from(const param::Map& params);
};

class ProjectiveIntegrator : public IntegratorBase {
 public:
  using Ptr = std::shared_ptr<ProjectiveIntegrator>;

  explicit ProjectiveIntegrator(
      const ProjectiveIntegratorConfig& config,
      ProjectorBase::ConstPtr projection_model,
      std::shared_ptr<PosedImage<>> posed_range_image,
      std::shared_ptr<Image<Vector2D>> beam_offset_image,
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
  std::shared_ptr<const PosedImage<>> getPosedRangeImage() const {
    return posed_range_image_;
  }
  std::shared_ptr<const Image<Vector2D>> getBeamOffsetImage() const {
    return beam_offset_image_;
  }

 protected:
  const ProjectiveIntegratorConfig config_;

  const ProjectorBase::ConstPtr projection_model_;
  const std::shared_ptr<PosedImage<>> posed_range_image_;
  const std::shared_ptr<Image<Vector2D>> beam_offset_image_;

  const MeasurementModelBase::ConstPtr measurement_model_;

  virtual void importPointcloud(const PosedPointcloud<Point3D>& pointcloud);
  virtual void importRangeImage(const PosedImage<>& range_image_input);

  virtual void updateMap() = 0;

  FloatingPoint computeUpdate(const Point3D& C_cell_center) const;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/impl/projective_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_PROJECTIVE_INTEGRATOR_H_
