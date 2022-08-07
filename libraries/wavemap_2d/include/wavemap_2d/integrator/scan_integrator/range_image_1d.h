#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_1D_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_1D_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common/integrator/circular_projector.h>

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
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_1D_H_
