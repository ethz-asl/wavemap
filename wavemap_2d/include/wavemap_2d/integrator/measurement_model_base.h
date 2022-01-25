#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BASE_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BASE_H_

#include <memory>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/datastructure_base.h"

namespace wavemap_2d {
class MeasurementModelBase {
 public:
  using Ptr = std::shared_ptr<MeasurementModelBase>;

  static constexpr FloatingPoint kRangeMax = 20.f;

  explicit MeasurementModelBase(FloatingPoint resolution)
      : resolution_(resolution),
        resolution_inv_(1.f / resolution),
        W_start_point_(Point::Zero()),
        W_end_point_(Point::Zero()),
        measured_distance_(0.f) {}
  virtual ~MeasurementModelBase() = default;

  void setStartPoint(const Point& start_point) {
    W_start_point_ = start_point;
    measured_distance_ = (W_end_point_ - W_start_point_).norm();
    updateCachedVariablesDerived();
  }
  void setEndPoint(const Point& end_point) {
    W_end_point_ = end_point;
    measured_distance_ = (W_end_point_ - W_start_point_).norm();
    updateCachedVariablesDerived();
  }

  FloatingPoint getLength() const { return measured_distance_; }
  bool exceedsMaxRange() const { return kRangeMax < measured_distance_; }
  Point getEndPointOrMaxRange() const {
    if (kRangeMax < measured_distance_) {
      return kRangeMax / measured_distance_ * (W_end_point_ - W_start_point_);
    } else {
      return W_end_point_;
    }
  }

  virtual Index getBottomLeftUpdateIndex() const = 0;
  virtual Index getTopRightUpdateIndex() const = 0;

  virtual FloatingPoint computeUpdateAt(const Index& index) const = 0;
  virtual void updateMap(DataStructureBase& map) const = 0;

 protected:
  static constexpr bool kUseClearing = true;

  const FloatingPoint resolution_;
  const FloatingPoint resolution_inv_;

  Point W_start_point_;
  Point W_end_point_;
  FloatingPoint measured_distance_;

  virtual void updateCachedVariablesDerived() {}
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BASE_H_
