#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_1D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_1D_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common/integrator/projection_model/image_1d/circular_projector.h>

namespace wavemap {
class RangeImage1D {
 public:
  using Data = Eigen::Matrix<FloatingPoint, 1, Eigen::Dynamic>;

  explicit RangeImage1D(IndexElement num_beams,
                        FloatingPoint initial_value = 0.f)
      : initial_value_(initial_value),
        data_(Data::Constant(1, num_beams, initial_value)) {}

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(IndexElement num_beams) { data_.resize(1, num_beams); }
  void clear() { resize(0); }

  void setToConstant(FloatingPoint value) { data_.setConstant(value); }
  void resetToInitialValue() { setToConstant(initial_value_); }

  IndexElement getNumBeams() const {
    return static_cast<IndexElement>(data_.cols());
  }
  const Data& getData() const { return data_; }

  bool isIndexWithinBounds(const IndexElement& index) const {
    return 0 <= index && index < getNumBeams();
  }

  FloatingPoint& getRange(IndexElement index) {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, data_.cols());
    return data_(0, index);
  }
  const FloatingPoint& getRange(IndexElement index) const {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, data_.cols());
    return data_(0, index);
  }

 private:
  const FloatingPoint initial_value_;
  Data data_;
};

class PosedRangeImage1D : public RangeImage1D {
 public:
  using RangeImage1D::RangeImage1D;

  void setPose(const Transformation2D& T_W_C) {
    T_W_C_ = T_W_C;
    T_C_W_ = T_W_C.inverse();
  }
  const Transformation2D& getPose() const { return T_W_C_; }
  const Transformation2D& getPoseInverse() const { return T_C_W_; }

 private:
  Transformation2D T_W_C_;
  Transformation2D T_C_W_;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_1D_H_
