#ifndef WAVEMAP_2D_ITERATOR_POINTCLOUD_ITERATOR_H_
#define WAVEMAP_2D_ITERATOR_POINTCLOUD_ITERATOR_H_

#include <utility>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
template <typename PointcloudType, int point_dimensions>
class PointcloudIterator {
 public:
  using PointcloudData =
      Eigen::Matrix<FloatingPoint, point_dimensions, Eigen::Dynamic>;
  using difference_type = std::ptrdiff_t;
  using value_type = Point;
  using pointer = void;
  using reference = std::conditional_t<std::is_const_v<PointcloudType>,
                                       typename PointcloudData::ConstColXpr,
                                       typename PointcloudData::ColXpr>;
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
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ITERATOR_POINTCLOUD_ITERATOR_H_
