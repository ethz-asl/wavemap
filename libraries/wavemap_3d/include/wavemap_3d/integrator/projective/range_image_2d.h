#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_2D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_2D_H_

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

class PosedRangeImage2D : public RangeImage2D {
 public:
  using RangeImage2D::RangeImage2D;

  PosedRangeImage2D(
      const PosedPointcloud<Point3D, Transformation3D>& posed_pointcloud,
      const SphericalProjector& spherical_projector)
      : RangeImage2D(posed_pointcloud.getPointsLocal(), spherical_projector) {
    setPose(posed_pointcloud.getPose());
  }

  void importPointcloud(
      const PosedPointcloud<Point3D, Transformation3D>& posed_pointcloud,
      const SphericalProjector& spherical_projector) {
    RangeImage2D::importPointcloud(posed_pointcloud.getPointsLocal(),
                                   spherical_projector);
    setPose(posed_pointcloud.getPose());
  }

  void setPose(const Transformation3D& T_W_C) {
    T_W_C_ = T_W_C;
    T_C_W_ = T_W_C.inverse();
  }
  const Transformation3D& getPose() const { return T_W_C_; }
  const Transformation3D& getPoseInverse() const { return T_C_W_; }

 private:
  Transformation3D T_W_C_;
  Transformation3D T_C_W_;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_2D_H_
