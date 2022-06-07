#ifndef WAVEMAP_2D_ITERATOR_GRID_ITERATOR_H_
#define WAVEMAP_2D_ITERATOR_GRID_ITERATOR_H_

#include <utility>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"

namespace wavemap_2d {
class Grid {
 public:
  Grid(Index min_index, const Index& max_index)
      : min_index_(std::move(min_index)),
        max_index_(max_index + Index::Ones()) {}

  class Iterator {
   public:
    explicit Iterator(const Grid& grid)
        : grid_(grid),
          current_index_(grid_.min_index_.x(), grid_.min_index_.y()) {}
    Iterator(const Grid& grid, bool end)
        : grid_(grid),
          current_index_(end ? grid_.max_index_.x() : grid_.min_index_.x(),
                         grid_.min_index_.y()) {}
    Index operator*() const { return current_index_; }
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
    Index current_index_;
  };

  Iterator begin() const { return Iterator{*this}; }
  Iterator end() const { return Iterator{*this, /*end*/ true}; }

 private:
  const Index min_index_;
  const Index max_index_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ITERATOR_GRID_ITERATOR_H_
