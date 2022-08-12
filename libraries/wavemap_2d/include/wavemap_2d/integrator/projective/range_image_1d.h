#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_1D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_RANGE_IMAGE_1D_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common/integrator/projection_model/circular_projector.h>

namespace wavemap {
class RangeImage1D {
 public:
  using Data = Eigen::Matrix<FloatingPoint, 1, Eigen::Dynamic>;

  explicit RangeImage1D(IndexElement num_beams)
      : data_(Data::Zero(1, num_beams)) {}

  RangeImage1D(const Pointcloud<Point2D>& pointcloud,
               const CircularProjector& circular_projector)
      : RangeImage1D(circular_projector.getNumCells()) {
    importPointcloud(pointcloud, circular_projector);
  }

  void importPointcloud(const Pointcloud<Point2D>& pointcloud,
                        const CircularProjector& circular_projector);

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(IndexElement num_beams) { data_.resize(1, num_beams); }
  void clear() { resize(0); }

  IndexElement getNumBeams() const {
    return static_cast<IndexElement>(data_.cols());
  }
  const Data& getData() const { return data_; }

  FloatingPoint& operator[](IndexElement index) {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, data_.cols());
    return data_(0, index);
  }
  const FloatingPoint& operator[](IndexElement index) const {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, data_.cols());
    return data_(0, index);
  }

 private:
  Data data_;
};

class PosedRangeImage1D : public RangeImage1D {
 public:
  using RangeImage1D::RangeImage1D;

  PosedRangeImage1D(
      const PosedPointcloud<Point2D, Transformation2D>& posed_pointcloud,
      const CircularProjector& circular_projector)
      : RangeImage1D(posed_pointcloud.getPointsLocal(), circular_projector) {
    setPose(posed_pointcloud.getPose());
  }

  void importPointcloud(
      const PosedPointcloud<Point2D, Transformation2D>& posed_pointcloud,
      const CircularProjector& circular_projector) {
    RangeImage1D::importPointcloud(posed_pointcloud.getPointsLocal(),
                                   circular_projector);
    setPose(posed_pointcloud.getPose());
  }

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
