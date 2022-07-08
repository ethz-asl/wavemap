#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_POINTCLOUD_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_POINTCLOUD_H_

#include <utility>

#include "wavemap_common/common.h"
#include "wavemap_common/iterator/pointcloud_iterator.h"

namespace wavemap {
template <typename PointT>
class Pointcloud {
 public:
  static constexpr int kPointDim = dim<PointT>;
  using PointType = PointT;
  using PointcloudData =
      Eigen::Matrix<FloatingPoint, kPointDim, Eigen::Dynamic>;

  Pointcloud() = default;
  explicit Pointcloud(PointcloudData pointcloud)
      : data_(std::move(pointcloud)) {}

  template <typename PointContainer>
  explicit Pointcloud(const PointContainer& point_container) {
    data_.resize(kPointDim, point_container.size());
    Eigen::Index column_idx = 0;
    for (const auto& point : point_container) {
      data_.col(column_idx++) = point;
    }
  }

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(const unsigned int n_points) {
    data_.resize(kPointDim, n_points);
  }
  void clear() { data_.resize(kPointDim, 0); }

  typename PointcloudData::ColXpr operator[](Eigen::Index point_index) {
    return data_.col(point_index);
  }
  typename PointcloudData::ConstColXpr operator[](
      Eigen::Index point_index) const {
    return data_.col(point_index);
  }

  PointcloudData& data() { return data_; }
  const PointcloudData& data() const { return data_; }

  using iterator = PointcloudIterator<Pointcloud, kPointDim>;
  using const_iterator = PointcloudIterator<const Pointcloud, kPointDim>;
  iterator begin() { return iterator(*this); }
  iterator end() { return iterator(*this, data_.cols()); }
  const_iterator begin() const { return cbegin(); }
  const_iterator end() const { return cend(); }
  const_iterator cbegin() const { return const_iterator(*this); }
  const_iterator cend() const { return const_iterator(*this, data_.cols()); }

 private:
  PointcloudData data_;
};

template <typename PointT, typename PoseT>
class PosedPointcloud {
 public:
  static constexpr int kPointDim = dim<PointT>;
  using PointType = PointT;
  using PoseType = PoseT;

  PosedPointcloud() = default;
  PosedPointcloud(const PoseType& T_W_C, Pointcloud<PointType> points_C)
      : T_W_C_(T_W_C), points_C_(std::move(points_C)) {}

  bool empty() const { return !size(); }
  size_t size() const { return points_C_.size(); }

  const typename PoseType::Position& getOrigin() const {
    return T_W_C_.getPosition();
  }
  const PoseType& getPose() const { return T_W_C_; }

  const Pointcloud<PointType>& getPointsLocal() const { return points_C_; }
  Pointcloud<PointType> getPointsGlobal() const {
    return static_cast<Pointcloud<PointType>>(
        T_W_C_.transformVectorized(points_C_.data()));
  }

 private:
  PoseType T_W_C_;
  Pointcloud<PointType> points_C_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_POINTCLOUD_H_
