#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common/integrator/circle_projector.h>

namespace wavemap {
class RangeImage {
 public:
  using RangeImageData = Eigen::Matrix<FloatingPoint, 1, Eigen::Dynamic>;

  explicit RangeImage(IndexElement num_beams)
      : data_(RangeImageData::Zero(1, num_beams)) {}

  RangeImage(const Pointcloud<Point2D>& pointcloud,
             const CircleProjector& circle_projector)
      : RangeImage(circle_projector.getNumCells()) {
    importPointcloud(pointcloud, circle_projector);
  }

  void importPointcloud(const Pointcloud<Point2D>& pointcloud,
                        const CircleProjector& circle_projector);

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(const unsigned int n_points) { data_.resize(1, n_points); }
  void clear() { data_.resize(1, 0); }

  IndexElement getNumBeams() const {
    return static_cast<IndexElement>(data_.cols());
  }
  const RangeImageData& getData() const { return data_; }

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
  RangeImageData data_;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_H_
