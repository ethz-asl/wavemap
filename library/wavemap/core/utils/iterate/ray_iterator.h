#ifndef WAVEMAP_UTILS_ITERATE_RAY_ITERATOR_H_
#define WAVEMAP_UTILS_ITERATE_RAY_ITERATOR_H_

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/index_conversions.h"

namespace wavemap {
// NOTE: The ray casting code in this class is largely based on voxblox's
//       RayCaster class, please see:
//       https://github.com/ethz-asl/voxblox/blob/master/voxblox/include/voxblox/integrator/integrator_utils.h#L100
template <int dim>
class Ray {
 public:
  Ray(const Index<dim>& start_index, const Index<dim>& end_index);
  Ray(const Point<dim>& start_point, const Point<dim>& end_point,
      FloatingPoint cell_width);

  size_t size() const { return ray_length_in_steps_; }

  class Iterator {
   public:
    explicit Iterator(const Ray& ray)
        : ray_(ray),
          current_index_(ray_.start_index_),
          t_to_next_boundary_(ray_.t_to_next_boundary_init_) {}
    Iterator(const Ray& ray, bool end) : Iterator(ray) {
      if (end) {
        current_step_ = ray_.ray_length_in_steps_;
        current_index_ = ray_.end_index_;
      }
    }

    Index<dim> operator*() const { return current_index_; }
    Iterator& operator++();     // prefix ++
    Iterator operator++(int) {  // postfix ++
      Iterator retval = *this;
      ++(*this);  // call the above prefix incrementer
      return retval;
    }
    friend bool operator==(const Iterator& lhs, const Iterator& rhs) {
      return (lhs.current_step_ == rhs.current_step_) &&
             (&lhs.ray_ == &rhs.ray_);
    }
    friend bool operator!=(const Iterator& lhs, const Iterator& rhs) {
      return !(lhs == rhs);  // NOLINT
    }

   private:
    const Ray& ray_;
    unsigned int current_step_ = 0u;
    Index<dim> current_index_;
    Vector<dim> t_to_next_boundary_;
  };

  Iterator begin() const { return Iterator{*this}; }
  Iterator end() const { return Iterator{*this, /*end*/ true}; }

 private:
  const Index<dim> start_index_;
  const Index<dim> end_index_;
  const size_t ray_length_in_steps_;

  Index<dim> ray_step_signs_;
  Vector<dim> t_step_size_;
  Vector<dim> t_to_next_boundary_init_;
};
}  // namespace wavemap

#include "wavemap/core/utils/iterate/impl/ray_iterator_inl.h"

#endif  // WAVEMAP_UTILS_ITERATE_RAY_ITERATOR_H_
