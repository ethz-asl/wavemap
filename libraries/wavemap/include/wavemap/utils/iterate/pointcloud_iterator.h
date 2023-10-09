#ifndef WAVEMAP_UTILS_ITERATE_POINTCLOUD_ITERATOR_H_
#define WAVEMAP_UTILS_ITERATE_POINTCLOUD_ITERATOR_H_

#include <utility>

#include "wavemap/common.h"

namespace wavemap {
template <typename PointcloudType, int point_dimensions>
class PointcloudIterator {
 public:
  using PointcloudData =
      Eigen::Matrix<FloatingPoint, point_dimensions, Eigen::Dynamic>;
  using difference_type = std::ptrdiff_t;
  using value_type = Point<point_dimensions>;
  using pointer = void;
  using reference = std::conditional_t<std::is_const_v<PointcloudType>,
                                       typename PointcloudData::ConstColXpr,
                                       typename PointcloudData::ColXpr>;
  using iterator_category = std::forward_iterator_tag;
  // NOTE: This iterator does not expose pointers to its values (only
  //       references) since pointers wouldn't play nice with Eigen

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

 private:
  PointcloudType& pointcloud_;
  Eigen::Index index_;
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_ITERATE_POINTCLOUD_ITERATOR_H_
