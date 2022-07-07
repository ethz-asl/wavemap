#ifndef WAVEMAP_2D_ITERATOR_GRID_ITERATOR_H_
#define WAVEMAP_2D_ITERATOR_GRID_ITERATOR_H_

#include <utility>

#include <wavemap_common/common.h>

namespace wavemap {
class Grid {
 public:
  Grid(Index2D min_index, const Index2D& max_index)
      : min_index_(std::move(min_index)),
        max_index_(max_index + Index2D::Ones()) {}

  class Iterator {
   public:
    using difference_type = std::ptrdiff_t;
    using value_type = Index2D;
    using pointer = Index2D*;
    using reference = Index2D&;
    using iterator_category = std::forward_iterator_tag;

    explicit Iterator(const Grid& grid)
        : grid_(grid),
          current_index_(grid_.min_index_.x(), grid_.min_index_.y()) {}
    Iterator(const Grid& grid, bool end)
        : grid_(grid),
          current_index_(end ? grid_.max_index_.x() : grid_.min_index_.x(),
                         grid_.min_index_.y()) {}

    const Index2D& operator*() const { return current_index_; }
    Iterator& operator++() {  // prefix ++
      if (++current_index_.y() == grid_.max_index_.y()) {
        current_index_.y() = grid_.min_index_.y();
        ++current_index_.x();
      }
      return *this;
    }
    Iterator operator++(int) {  // postfix ++
      Iterator retval = *this;
      ++(*this);  // call the above prefix incrementer
      return retval;
    }
    friend bool operator==(const Iterator& lhs, const Iterator& rhs) {
      return lhs.current_index_ == rhs.current_index_;
    }
    friend bool operator!=(const Iterator& lhs, const Iterator& rhs) {
      return !(lhs == rhs);  // NOLINT
    }

   private:
    const Grid& grid_;
    Index2D current_index_;
  };

  Iterator begin() const { return Iterator{*this}; }
  Iterator end() const { return Iterator{*this, /*end*/ true}; }

 private:
  const Index2D min_index_;
  const Index2D max_index_;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_ITERATOR_GRID_ITERATOR_H_
