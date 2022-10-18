#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_BEARING_IMAGE_2D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_BEARING_IMAGE_2D_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common/integrator/projection_model/image_2d/ouster_projector.h>

namespace wavemap {
class BearingImage2D {
 public:
  using Data = Eigen::Matrix<Vector3D, Eigen::Dynamic, Eigen::Dynamic>;

  explicit BearingImage2D(const Index2D& dimensions)
      : BearingImage2D(dimensions.x(), dimensions.y()) {}
  BearingImage2D(IndexElement num_rows, IndexElement num_columns)
      : data_(Data::Constant(num_rows, num_columns, Vector3D::Zero())) {}

  bool empty() const { return !size(); }
  size_t size() const { return data_.size(); }
  void resize(IndexElement num_rows, IndexElement num_columns) {
    data_.resize(num_rows, num_columns);
  }
  void clear() { resize(0, 0); }

  void setToConstant(const Vector3D& value) { data_.setConstant(value); }
  void resetToInitialValue() { setToConstant(Vector3D::Zero()); }

  IndexElement getNumRows() const {
    return static_cast<IndexElement>(data_.rows());
  }
  IndexElement getNumColumns() const {
    return static_cast<IndexElement>(data_.cols());
  }
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }
  const Data& getData() const { return data_; }

  Vector3D& getBearing(Index2D index) {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }
  const Vector3D& getBearing(Index2D index) const {
    DCHECK((0 <= index.array()).all());
    DCHECK_LT(index.x(), data_.rows());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }

 private:
  Data data_;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_BEARING_IMAGE_2D_H_
