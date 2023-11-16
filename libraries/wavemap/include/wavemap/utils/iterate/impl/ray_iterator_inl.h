#ifndef WAVEMAP_UTILS_ITERATE_IMPL_RAY_ITERATOR_INL_H_
#define WAVEMAP_UTILS_ITERATE_IMPL_RAY_ITERATOR_INL_H_

namespace wavemap {
template <int dim>
Ray<dim>::Ray(const Index<dim>& start_index, const Index<dim>& end_index)
    : start_index_(start_index),
      end_index_(end_index),
      ray_length_in_steps_((end_index_ - start_index_).cwiseAbs().sum() + 1u) {
  if (ray_length_in_steps_ == 0u) {
    return;
  }

  // Compute the direction of steps along the ray
  const Index<dim> ray_discrete = end_index_ - start_index_;
  ray_step_signs_ = ray_discrete.cwiseSign();

  // Compute distance to cross one grid cell along the ray in t
  const Vector<dim> ray = ray_discrete.template cast<FloatingPoint>();
  t_step_size_ =
      (ray_discrete.array() == 0)
          .select(2.f,
                  ray_step_signs_.template cast<FloatingPoint>().cwiseQuotient(
                      ray));

  // Compute the initial value for the traversal in t
  const Index<dim> step_rectified = ray_step_signs_.cwiseMax(0);
  const Vector<dim> distance_to_boundaries =
      step_rectified.template cast<FloatingPoint>().array() - 0.5f;
  t_to_next_boundary_init_ =
      (ray_discrete.array() == 0)
          .select(2.f, distance_to_boundaries.cwiseQuotient(ray));
}

template <int dim>
Ray<dim>::Ray(const Point<dim>& start_point, const Point<dim>& end_point,
              FloatingPoint cell_width)
    : start_index_(convert::pointToNearestIndex(start_point, 1.f / cell_width)),
      end_index_(convert::pointToNearestIndex(end_point, 1.f / cell_width)),
      ray_length_in_steps_((end_index_ - start_index_).cwiseAbs().sum() + 1u) {
  CHECK(!start_point.hasNaN());
  CHECK(!end_point.hasNaN());
  if (ray_length_in_steps_ == 0u) {
    return;
  }

  // Compute the direction of steps along the ray
  const Vector<dim> ray = end_point - start_point;
  ray_step_signs_ = ray.cwiseSign().template cast<IndexElement>();

  // Compute distance to cross one grid cell along the ray in t
  t_step_size_ =
      (ray.array() == 0.f)
          .select(2.f, convert::indexToMinCorner(ray_step_signs_, cell_width)
                           .cwiseQuotient(ray));

  // Compute the initial value for the traversal in t
  const Index<dim> step_rectified = ray_step_signs_.cwiseMax(0);
  const Point<dim> start_relative =
      start_point - convert::indexToMinCorner(start_index_, cell_width);
  const Vector<dim> distance_to_boundaries =
      convert::indexToMinCorner(step_rectified, cell_width) - start_relative;
  t_to_next_boundary_init_ =
      (ray.array() == 0.f)
          .select(2.f, distance_to_boundaries.cwiseQuotient(ray));
}

template <int dim>
typename Ray<dim>::Iterator& Ray<dim>::Iterator::operator++() {
  int t_min_idx;
  t_to_next_boundary_.minCoeff(&t_min_idx);

  current_index_[t_min_idx] += ray_.ray_step_signs_[t_min_idx];
  t_to_next_boundary_[t_min_idx] += ray_.t_step_size_[t_min_idx];
  ++current_step_;

  return *this;
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_ITERATE_IMPL_RAY_ITERATOR_INL_H_
