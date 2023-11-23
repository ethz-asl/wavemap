#ifndef WAVEMAP_DATA_STRUCTURE_POINTCLOUD_H_
#define WAVEMAP_DATA_STRUCTURE_POINTCLOUD_H_

#include <limits>
#include <utility>

#include "wavemap/common.h"
#include "wavemap/data_structure/posed_object.h"
#include "wavemap/utils/iterate/pointcloud_iterator.h"

namespace wavemap {
template <typename PointT = Point3D>
class Pointcloud {
 public:
  static constexpr int kDim = dim_v<PointT>;
  using PointType = PointT;
  using PointcloudData = Eigen::Matrix<FloatingPoint, kDim, Eigen::Dynamic>;

  Pointcloud() = default;
  explicit Pointcloud(PointcloudData pointcloud)
      : data_(std::move(pointcloud)) {}

  template <typename PointContainer>
  explicit Pointcloud(const PointContainer& point_container) {
    data_.resize(kDim, point_container.size());
    Eigen::Index column_idx = 0;
    for (const auto& point : point_container) {
      data_.col(column_idx++) = point;
    }
  }

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(size_t n_points) {
    CHECK_LT(n_points, std::numeric_limits<Eigen::Index>::max());
    data_.resize(kDim, n_points);
  }
  void clear() { data_.resize(kDim, 0); }

  typename PointcloudData::ColXpr operator[](Eigen::Index point_index) {
    return data_.col(point_index);
  }
  typename PointcloudData::ConstColXpr operator[](
      Eigen::Index point_index) const {
    return data_.col(point_index);
  }

  PointcloudData& data() { return data_; }
  const PointcloudData& data() const { return data_; }

  using iterator = PointcloudIterator<Pointcloud, kDim>;
  using const_iterator = PointcloudIterator<const Pointcloud, kDim>;
  iterator begin() { return iterator(*this); }
  iterator end() { return iterator(*this, data_.cols()); }
  const_iterator begin() const { return cbegin(); }
  const_iterator end() const { return cend(); }
  const_iterator cbegin() const { return const_iterator(*this); }
  const_iterator cend() const { return const_iterator(*this, data_.cols()); }

 private:
  PointcloudData data_;
};

template <typename PointT = Point3D>
class PosedPointcloud : public PosedObject<Pointcloud<PointT>> {
 public:
  using PointType = PointT;
  using PointcloudType = Pointcloud<PointType>;

  using PosedObject<Pointcloud<PointT>>::PosedObject;

  const PointcloudType& getPointsLocal() const { return *this; }

  PointcloudType getPointsGlobal() const {
    return static_cast<PointcloudType>(
        Base::getPose().transformVectorized(this->data()));
  }

 private:
  using Base = PosedObject<Pointcloud<PointT>>;
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_POINTCLOUD_H_
