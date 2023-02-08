#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_2D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_2D_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>

namespace wavemap {
class RangeImage2D {
 public:
  using Data = Eigen::Matrix<FloatingPoint, Eigen::Dynamic, Eigen::Dynamic>;

  explicit RangeImage2D(const Index2D& dimensions,
                        FloatingPoint initial_value = 0.f)
      : RangeImage2D(dimensions.x(), dimensions.y(), initial_value) {}
  RangeImage2D(IndexElement num_rows, IndexElement num_columns,
               FloatingPoint initial_value = 0.f)
      : initial_value_(initial_value),
        data_(Data::Constant(num_rows, num_columns, initial_value)) {}

  bool empty() const { return !size(); }
  size_t size() const { return data_.size(); }
  void resize(IndexElement num_rows, IndexElement num_columns) {
    data_.resize(num_rows, num_columns);
  }
  void clear() { resize(0, 0); }

  void setToConstant(FloatingPoint value) { data_.setConstant(value); }
  void resetToInitialValue() { setToConstant(initial_value_); }

  IndexElement getNumRows() const {
    return static_cast<IndexElement>(data_.rows());
  }
  IndexElement getNumColumns() const {
    return static_cast<IndexElement>(data_.cols());
  }
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }
  Data& getData() { return data_; }
  const Data& getData() const { return data_; }

  bool isIndexWithinBounds(const Index2D& index) const {
    return (0 <= index.array() && index.array() < getDimensions().array())
        .all();
  }

  FloatingPoint& getRange(Index2D index) {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }
  const FloatingPoint& getRange(Index2D index) const {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }

 private:
  const FloatingPoint initial_value_;
  Data data_;
};

class PosedRangeImage2D : public RangeImage2D {
 public:
  using RangeImage2D::RangeImage2D;

  void setPose(const Transformation3D& T_W_C) {
    T_W_C_ = T_W_C;
    R_W_C_ = T_W_C_.getRotationMatrix();
    T_C_W_ = T_W_C.inverse();
    R_C_W_ = T_C_W_.getRotationMatrix();
  }
  const Transformation3D& getPose() const { return T_W_C_; }
  const Transformation3D& getPoseInverse() const { return T_C_W_; }
  const Transformation3D::RotationMatrix& getRotationMatrix() const {
    return R_W_C_;
  }
  const Transformation3D::RotationMatrix& getRotationMatrixInverse() const {
    return R_C_W_;
  }

 private:
  Transformation3D T_W_C_;
  Transformation3D T_C_W_;
  Transformation3D::RotationMatrix R_W_C_;
  Transformation3D::RotationMatrix R_C_W_;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_2D_H_
