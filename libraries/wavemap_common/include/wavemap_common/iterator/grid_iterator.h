#ifndef WAVEMAP_COMMON_ITERATOR_GRID_ITERATOR_H_
#define WAVEMAP_COMMON_ITERATOR_GRID_ITERATOR_H_

#include <utility>

#include "wavemap_common/common.h"

namespace wavemap {
template <int dim>
class Grid {
 public:
  Grid(Index<dim> min_index, Index<dim> max_index)
      : min_index_(std::move(min_index)), max_index_(std::move(max_index)) {}

  class Iterator {
   public:
    using difference_type = std::ptrdiff_t;
    using value_type = Index<dim>;
    using pointer = Index<dim>*;
    using reference = Index<dim>&;
    using iterator_category = std::forward_iterator_tag;

    explicit Iterator(const Grid& grid)
        : grid_(grid), current_index_(grid_.min_index_) {}
    Iterator(const Grid& grid, bool end)
        : grid_(grid),
          current_index_(
              end ? Index<dim>::Unit(dim - 1).select(
                        grid_.max_index_ + Index<dim>::Ones(), grid_.min_index_)
                  : grid_.min_index_) {}

    const Index<dim>& operator*() const { return current_index_; }
    Iterator& operator++() {  // prefix ++
      ++current_index_[0];
      for (int dim_idx = 0; dim_idx < dim - 1; ++dim_idx) {
        if (grid_.max_index_[dim_idx] < current_index_[dim_idx]) {
          current_index_[dim_idx] = grid_.min_index_[dim_idx];
          ++current_index_[dim_idx + 1];
        }
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
    Index<dim> current_index_;
  };

  Iterator begin() const { return Iterator{*this}; }
  Iterator end() const { return Iterator{*this, /*end*/ true}; }

 private:
  const Index<dim> min_index_;
  const Index<dim> max_index_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_ITERATOR_GRID_ITERATOR_H_
