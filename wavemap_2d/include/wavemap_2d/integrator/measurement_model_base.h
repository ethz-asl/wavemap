#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BASE_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BASE_H_

namespace wavemap_2d {
class MeasurementModelBase {
 public:
  static constexpr FloatingPoint kRangeMax = 40.f;

  explicit MeasurementModelBase(FloatingPoint resolution)
      : resolution_(resolution),
        resolution_inv_(1.f / resolution),
        measured_distance_(0.f) {}

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
  virtual Index getBottomLeftUpdateIndex() = 0;
  virtual Index getTopRightUpdateIndex() = 0;

  virtual FloatingPoint computeUpdateAt(const Index& index) = 0;

 protected:
  const FloatingPoint resolution_;
  const FloatingPoint resolution_inv_;

  Point W_start_point_;
  Point W_end_point_;
  FloatingPoint measured_distance_;

  virtual void updateCachedVariablesDerived() {}
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_BASE_H_
