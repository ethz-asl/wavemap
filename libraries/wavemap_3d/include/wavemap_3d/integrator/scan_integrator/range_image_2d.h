#ifndef WAVEMAP_3D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_2D_H_
#define WAVEMAP_3D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_2D_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common/integrator/projection_model/spherical_projector.h>

namespace wavemap {
class RangeImage2D {
 public:
  using Data = Eigen::Matrix<FloatingPoint, Eigen::Dynamic, Eigen::Dynamic>;

  explicit RangeImage2D(IndexElement num_rows, IndexElement num_columns)
      : data_(Data::Zero(num_rows, num_columns)) {}

  RangeImage2D(const Pointcloud<Point3D>& pointcloud,
               const SphericalProjector& spherical_projector)
      : RangeImage2D(spherical_projector.getNumRows(),
                     spherical_projector.getNumColumns()) {
    importPointcloud(pointcloud, spherical_projector);
  }

  void importPointcloud(const Pointcloud<Point3D>& pointcloud,
                        const SphericalProjector& spherical_projector);

  bool empty() const { return !size(); }
  size_t size() const { return data_.size(); }
  void resize(IndexElement num_rows, IndexElement num_columns) {
    data_.resize(num_rows, num_columns);
  }
  void clear() { resize(0, 0); }

  IndexElement getNumRows() const {
    return static_cast<IndexElement>(data_.rows());
  }
  IndexElement getNumColumns() const {
    return static_cast<IndexElement>(data_.cols());
  }
  const Data& getData() const { return data_; }

  FloatingPoint& operator[](Index2D index) {
    DCHECK((0 < index.array()).all());
    DCHECK_LT(index.x(), data_.cols());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }
  const FloatingPoint& operator[](Index2D index) const {
    DCHECK((0 < index.array()).all());
    DCHECK_LT(index.x(), data_.cols());
    DCHECK_LT(index.y(), data_.cols());
    return data_(index.x(), index.y());
  }

 private:
  Data data_;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_2D_H_
