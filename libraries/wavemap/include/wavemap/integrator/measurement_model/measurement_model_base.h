#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_BASE_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_BASE_H_

#include <memory>

#include "wavemap/common.h"
#include "wavemap/config/type_selector.h"
#include "wavemap/data_structure/image.h"
#include "wavemap/integrator/projection_model/projector_base.h"
#include "wavemap/integrator/projective/update_type.h"

namespace wavemap {
struct MeasurementModelType : TypeSelector<MeasurementModelType> {
  using TypeSelector<MeasurementModelType>::TypeSelector;

  enum Id : TypeId { kConstantRay, kContinuousRay, kContinuousBeam };

  static constexpr std::array names = {"constant_ray", "continuous_ray",
                                       "continuous_beam"};
};

struct BeamSelectorType : TypeSelector<BeamSelectorType> {
  using TypeSelector<BeamSelectorType>::TypeSelector;

  enum Id : TypeId { kNearestNeighbor, kAllNeighbors };

  static constexpr std::array names = {"nearest_neighbor", "all_neighbors"};
};

class MeasurementModelBase {
 public:
  using Ptr = std::shared_ptr<MeasurementModelBase>;
  using ConstPtr = std::shared_ptr<const MeasurementModelBase>;

  virtual FloatingPoint getPaddingAngle() const = 0;
  virtual FloatingPoint getPaddingSurfaceFront() const = 0;
  virtual FloatingPoint getPaddingSurfaceBack() const = 0;

  virtual FloatingPoint computeWorstCaseApproximationError(
      UpdateType update_type, FloatingPoint cell_to_sensor_distance,
      FloatingPoint cell_bounding_radius) const = 0;

  virtual FloatingPoint computeUpdate(
      const Vector3D& sensor_coordinates) const = 0;
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_BASE_H_
