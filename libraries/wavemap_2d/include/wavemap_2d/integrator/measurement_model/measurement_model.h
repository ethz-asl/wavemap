#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_H_

#include <memory>

#include <wavemap_common/common.h>

#include "wavemap_2d/data_structure/volumetric_data_structure.h"

namespace wavemap {
class MeasurementModel {
 public:
  using Ptr = std::shared_ptr<MeasurementModel>;

  static constexpr FloatingPoint kRangeMax = 20.f;

  explicit MeasurementModel(FloatingPoint min_cell_width)
      : min_cell_width_(min_cell_width),
        min_cell_width_inv_(1.f / min_cell_width) {}
  virtual ~MeasurementModel() = default;

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
  const Point& getStartPoint() const { return W_start_point_; }
  const Point& getEndPoint() const { return W_end_point_; }
  FloatingPoint getLength() const { return measured_distance_; }

  bool isMeasurementValid() const;
  bool exceedsMaxRange() const { return kRangeMax < measured_distance_; }
  Point getEndPointOrMaxRange() const {
    if (kRangeMax < measured_distance_) {
      return W_start_point_ +
             kRangeMax / measured_distance_ * (W_end_point_ - W_start_point_);
    } else {
      return W_end_point_;
    }
  }

  virtual Index getBottomLeftUpdateIndex() const = 0;
  virtual Index getTopRightUpdateIndex() const = 0;

  virtual FloatingPoint computeUpdateAt(const Index& index) const = 0;

 protected:
  static constexpr bool kUseClearing = true;

  const FloatingPoint min_cell_width_;
  const FloatingPoint min_cell_width_inv_;

  Point W_start_point_ = Point::Zero();
  Point W_end_point_ = Point::Zero();
  FloatingPoint measured_distance_ = 0.f;

  virtual void updateCachedVariablesDerived() {
    // NOTE: This method is called when the start and end points are updated. It
    //       can optionally be used by derived classes to update their dependent
    //       cached variables.
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_MEASUREMENT_MODEL_H_
