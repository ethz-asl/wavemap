#ifndef WAVEMAP_2D_POINTCLOUD_H_
#define WAVEMAP_2D_POINTCLOUD_H_

#include <utility>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
using PointcloudRaw = Eigen::Matrix<FloatingPoint, 2, Eigen::Dynamic>;

template <typename PointcloudType>
class PointcloudIterator {
 public:
  using Index = Eigen::Index;

  using iterator_category = std::forward_iterator_tag;
  using difference_type = std::ptrdiff_t;
  using value_type = Point;
  using reference = typename PointcloudType::ColXpr;
  // NOTE: This iterator does not expose pointers to its values (only
  //       references) since pointers wouldn't play nice with Eigen

  explicit PointcloudIterator(PointcloudType* pointcloud_ptr,
                              Index index = std::ptrdiff_t(0))
      : pointcloud_ptr_(pointcloud_ptr), index_(index) {}

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
    return (lhs.pointcloud_ptr_ == rhs.pointcloud_ptr_) &&
           (lhs.index_ == rhs.index_);
  }
  friend bool operator!=(const PointcloudIterator& lhs,
                         const PointcloudIterator& rhs) {
    return !(lhs == rhs);  // NOLINT
  }

  reference operator*() const { return pointcloud_ptr_->col(index_); }

 protected:
  PointcloudType* pointcloud_ptr_;
  Index index_;
};

class Pointcloud : public PointcloudRaw {
 public:
  Pointcloud() = default;
  explicit Pointcloud(PointcloudRaw pointcloud)
      : PointcloudRaw(std::move(pointcloud)) {}

  template <typename PointContainer>
  explicit Pointcloud(const PointContainer& point_container) {
    resize(2, point_container.size());
    Eigen::Index column_idx = 0;
    for (const auto& point : point_container) {
      col(column_idx++) = point;
    }
  }

  using iterator = PointcloudIterator<Pointcloud>;
  using const_iterator = PointcloudIterator<const Pointcloud>;
  iterator begin() { return iterator(this); }
  iterator end() { return iterator(this, cols()); }
  const_iterator cbegin() const { return const_iterator(this); }
  const_iterator cend() const { return const_iterator(this, cols()); }
};

class PosedPointcloud {
 public:
  PosedPointcloud(const Transformation& T_W_C, Pointcloud points_C)
      : T_W_C_(T_W_C), points_C_(std::move(points_C)) {}

  const Transformation& getPose() const { return T_W_C_; }
  Point getOrigin() const { return T_W_C_.getPosition(); }

  const Pointcloud& getPointsLocal() const { return points_C_; }
  Pointcloud getPointsGlobal() const {
    return static_cast<Pointcloud>(T_W_C_.transformVectorized(points_C_));
  }

 protected:
  Transformation T_W_C_;
  Pointcloud points_C_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_POINTCLOUD_H_
