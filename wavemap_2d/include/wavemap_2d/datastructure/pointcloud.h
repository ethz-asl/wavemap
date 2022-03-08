#ifndef WAVEMAP_2D_DATASTRUCTURE_POINTCLOUD_H_
#define WAVEMAP_2D_DATASTRUCTURE_POINTCLOUD_H_

#include <utility>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
constexpr int kPointcloudPointDim = 2;
using PointcloudData =
    Eigen::Matrix<FloatingPoint, kPointcloudPointDim, Eigen::Dynamic>;

template <typename PointcloudType>
class PointcloudIterator {
 public:
  using difference_type = std::ptrdiff_t;
  using value_type = Point;
  using pointer = void;
  using reference =
      std::conditional_t<std::is_const_v<PointcloudType>,
                         PointcloudData::ConstColXpr, PointcloudData::ColXpr>;
  using iterator_category = std::forward_iterator_tag;
  // NOTE: This iterator does not expose pointers to its values (only
  //       references) since pointers wouldn't play nice with Eigen

  // TODO(victorr): Modify the linter to adhere to the new google style guide,
  //                which recommends passing by non-const reference instead of
  //                by pointer
  explicit PointcloudIterator(PointcloudType& pointcloud,
                              Eigen::Index index = std::ptrdiff_t(0))
      : pointcloud_(pointcloud), index_(index) {}

  PointcloudIterator& operator++() {  // prefix ++
    ++index_;
    return *this;
  }
  PointcloudIterator operator++(int) {  // postfix ++
    PointcloudIterator retval = *this;
    ++(*this);  // call the above prefix incrementer
    return retval;
  }

  friend bool operator==(const PointcloudIterator& lhs,
                         const PointcloudIterator& rhs) {
    return (&lhs.pointcloud_ == &rhs.pointcloud_) && (lhs.index_ == rhs.index_);
  }
  friend bool operator!=(const PointcloudIterator& lhs,
                         const PointcloudIterator& rhs) {
    return !(lhs == rhs);  // NOLINT
  }

  reference operator*() const { return pointcloud_.operator[](index_); }

 protected:
  PointcloudType& pointcloud_;
  Eigen::Index index_;
};

class Pointcloud {
 public:
  Pointcloud() = default;
  explicit Pointcloud(PointcloudData pointcloud)
      : data_(std::move(pointcloud)) {}

  template <typename PointContainer>
  explicit Pointcloud(const PointContainer& point_container) {
    data_.resize(kPointcloudPointDim, point_container.size());
    Eigen::Index column_idx = 0;
    for (const auto& point : point_container) {
      data_.col(column_idx++) = point;
    }
  }

  PointcloudData::ColXpr operator[](Eigen::Index point_index) {
    return data_.col(point_index);
  }
  PointcloudData::ConstColXpr operator[](Eigen::Index point_index) const {
    return data_.col(point_index);
  }

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }

  void clear() { data_.resize(wavemap_2d::kPointcloudPointDim, 0); }
  void resize(const unsigned int n_points) {
    data_.resize(wavemap_2d::kPointcloudPointDim, n_points);
  }

  PointcloudData& data() { return data_; }
  const PointcloudData& data() const { return data_; }

  using iterator = PointcloudIterator<Pointcloud>;
  using const_iterator = PointcloudIterator<const Pointcloud>;
  iterator begin() { return iterator(*this); }
  iterator end() { return iterator(*this, data_.cols()); }
  const_iterator begin() const { return cbegin(); }
  const_iterator end() const { return cend(); }
  const_iterator cbegin() const { return const_iterator(*this); }
  const_iterator cend() const { return const_iterator(*this, data_.cols()); }

 protected:
  PointcloudData data_;
};

class PosedPointcloud {
 public:
  PosedPointcloud() = default;
  PosedPointcloud(const Transformation& T_W_C, Pointcloud points_C)
      : T_W_C_(T_W_C), points_C_(std::move(points_C)) {}

  Point getOrigin() const { return T_W_C_.getPosition(); }
  const Transformation& getPose() const { return T_W_C_; }

  const Pointcloud& getPointsLocal() const { return points_C_; }
  Pointcloud getPointsGlobal() const {
    return static_cast<Pointcloud>(
        T_W_C_.transformVectorized(points_C_.data()));
  }

 protected:
  Transformation T_W_C_;
  Pointcloud points_C_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_POINTCLOUD_H_
