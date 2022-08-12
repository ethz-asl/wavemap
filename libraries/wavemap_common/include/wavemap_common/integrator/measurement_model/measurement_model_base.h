#ifndef WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_BASE_H_
#define WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_BASE_H_

#include <memory>

#include "wavemap_common/common.h"

namespace wavemap {
template <int dim>
class MeasurementModelBase {
 public:
  using Ptr = std::shared_ptr<MeasurementModelBase>;

  static constexpr FloatingPoint kRangeMax = 20.f;

  explicit MeasurementModelBase(FloatingPoint min_cell_width)
      : min_cell_width_(min_cell_width),
        min_cell_width_inv_(1.f / min_cell_width) {}
  virtual ~MeasurementModelBase() = default;

  void setStartPoint(const Point<dim>& start_point) {
    W_start_point_ = start_point;
    measured_distance_ = (W_end_point_ - W_start_point_).norm();
    updateCachedVariablesDerived();
  }
  void setEndPoint(const Point<dim>& end_point) {
    W_end_point_ = end_point;
    measured_distance_ = (W_end_point_ - W_start_point_).norm();
    updateCachedVariablesDerived();
  }
  const Point<dim>& getStartPoint() const { return W_start_point_; }
  const Point<dim>& getEndPoint() const { return W_end_point_; }
  FloatingPoint getMeasuredDistance() const { return measured_distance_; }

  bool isMeasurementValid() const;
  bool exceedsMaxRange() const { return kRangeMax < measured_distance_; }
  Point<dim> getEndPointOrMaxRange() const;

  // TODO(victorr): See if we can remove this from the base class
  virtual Index<dim> getBottomLeftUpdateIndex() const = 0;
  virtual Index<dim> getTopRightUpdateIndex() const = 0;

 protected:
  const FloatingPoint min_cell_width_;
  const FloatingPoint min_cell_width_inv_;

  Point<dim> W_start_point_ = Point<dim>::Zero();
  Point<dim> W_end_point_ = Point<dim>::Zero();
  FloatingPoint measured_distance_ = 0.f;

  // NOTE: This method is called when the start and end points are updated. It
  //       can optionally be used by derived classes to update their dependent
  //       cached variables.
  virtual void updateCachedVariablesDerived() {}
};
}  // namespace wavemap

#include "wavemap_common/integrator/measurement_model/impl/measurement_model_base_inl.h"

#endif  // WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_BASE_H_
