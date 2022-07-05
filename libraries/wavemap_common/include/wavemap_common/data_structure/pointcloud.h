#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_POINTCLOUD_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_POINTCLOUD_H_

#include <utility>

#include "wavemap_common/common.h"
#include "wavemap_common/iterator/pointcloud_iterator.h"

namespace wavemap {
template <int point_dimensions = 2>
class Pointcloud {
 public:
  static constexpr int kPointDimensions = point_dimensions;
  using PointcloudData =
      Eigen::Matrix<FloatingPoint, point_dimensions, Eigen::Dynamic>;

  Pointcloud() = default;
  explicit Pointcloud(PointcloudData pointcloud)
      : data_(std::move(pointcloud)) {}

  template <typename PointContainer>
  explicit Pointcloud(const PointContainer& point_container) {
    data_.resize(point_dimensions, point_container.size());
    Eigen::Index column_idx = 0;
    for (const auto& point : point_container) {
      data_.col(column_idx++) = point;
    }
  }

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(const unsigned int n_points) {
    data_.resize(point_dimensions, n_points);
  }
  void clear() { data_.resize(point_dimensions, 0); }

  typename PointcloudData::ColXpr operator[](Eigen::Index point_index) {
    return data_.col(point_index);
  }
  typename PointcloudData::ConstColXpr operator[](
      Eigen::Index point_index) const {
    return data_.col(point_index);
  }

  PointcloudData& data() { return data_; }
  const PointcloudData& data() const { return data_; }

  using iterator = PointcloudIterator<Pointcloud, point_dimensions>;
  using const_iterator = PointcloudIterator<const Pointcloud, point_dimensions>;
  iterator begin() { return iterator(*this); }
  iterator end() { return iterator(*this, data_.cols()); }
  const_iterator begin() const { return cbegin(); }
  const_iterator end() const { return cend(); }
  const_iterator cbegin() const { return const_iterator(*this); }
  const_iterator cend() const { return const_iterator(*this, data_.cols()); }

 private:
  PointcloudData data_;
};

template <int point_dimensions = 2>
class PosedPointcloud {
 public:
  PosedPointcloud() = default;
  PosedPointcloud(const Transformation& T_W_C,
                  Pointcloud<point_dimensions> points_C)
      : T_W_C_(T_W_C), points_C_(std::move(points_C)) {}

  bool empty() const { return !size(); }
  size_t size() const { return points_C_.size(); }

  const Point& getOrigin() const { return T_W_C_.getPosition(); }
  const Transformation& getPose() const { return T_W_C_; }

  const Pointcloud<point_dimensions>& getPointsLocal() const {
    return points_C_;
  }
  Pointcloud<point_dimensions> getPointsGlobal() const {
    return static_cast<Pointcloud<point_dimensions>>(
        T_W_C_.transformVectorized(points_C_.data()));
  }

 private:
  Transformation T_W_C_;
  Pointcloud<point_dimensions> points_C_;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_POINTCLOUD_H_
