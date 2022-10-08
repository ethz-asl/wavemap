#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_BOUNDS_2D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_BOUNDS_2D_INL_H_

#include <algorithm>
#include <vector>

#include <wavemap_common/iterator/grid_iterator.h>
#include <wavemap_common/utils/bit_manipulation.h>

namespace wavemap {
template <bool azimuth_wraps_pi>
inline Bounds<FloatingPoint>
HierarchicalRangeBounds2D<azimuth_wraps_pi>::getBounds(
    const QuadtreeIndex& index) const {
  DCHECK_GE(index.height, 0);
  DCHECK_LE(index.height, max_height_);
  if (index.height == 0) {
    const FloatingPoint range_image_value =
        range_image_->operator[](index.position);
    return {range_image_value, range_image_value};
  } else {
    return {lower_bound_levels_[index.height - 1][index.position],
            upper_bound_levels_[index.height - 1][index.position]};
  }
}

template <bool azimuth_wraps_pi>
inline Bounds<FloatingPoint>
HierarchicalRangeBounds2D<azimuth_wraps_pi>::getBounds(
    const Index2D& bottom_left_image_idx,
    const Index2D& top_right_image_idx) const {
  if (bottom_left_image_idx == top_right_image_idx) {
    const auto range = range_image_->operator[](bottom_left_image_idx);
    return {valueOrInit(range, kUnknownRangeImageValueLowerBound),
            valueOrInit(range, kUnknownRangeImageValueUpperBound)};
  }

  Index2D top_right_image_idx_unwrapped = top_right_image_idx;
  if (azimuth_wraps_pi && top_right_image_idx.y() < bottom_left_image_idx.y()) {
    top_right_image_idx_unwrapped.y() += range_image_->getNumColumns();
  }
  DCHECK(
      (bottom_left_image_idx.array() <= top_right_image_idx_unwrapped.array())
          .all());

  const Index2D bottom_left_image_idx_scaled = {
      std::get<0>(scale_) * bottom_left_image_idx.x(),
      std::get<1>(scale_) * bottom_left_image_idx.y()};
  const Index2D top_right_image_idx_scaled = {
      std::get<0>(scale_) * top_right_image_idx.x(),
      std::get<1>(scale_) * top_right_image_idx.y()};
  const Index2D top_right_image_idx_unwrapped_scaled = {
      std::get<0>(scale_) * top_right_image_idx_unwrapped.x(),
      std::get<1>(scale_) * top_right_image_idx_unwrapped.y()};

  const IndexElement max_idx_diff =
      (top_right_image_idx_unwrapped_scaled - bottom_left_image_idx_scaled)
          .maxCoeff();
  const IndexElement min_level_up =
      max_idx_diff == 0 ? 0 : int_math::log2_floor(max_idx_diff);

  // Compute the node indices at the minimum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D bottom_left_child_idx =
      int_math::div_exp2_floor(bottom_left_image_idx_scaled, min_level_up);
  const Index2D top_right_child_idx =
      int_math::div_exp2_floor(top_right_image_idx_scaled, min_level_up);
  const Index2D top_right_child_idx_unwrapped = int_math::div_exp2_floor(
      top_right_image_idx_unwrapped_scaled, min_level_up);
  const IndexElement child_height_idx = min_level_up - 1;

  // Compute the node indices at the maximum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D& bottom_left_parent_idx =
      int_math::div_exp2_floor(bottom_left_child_idx, 1);
  const Index2D top_right_parent_idx =
      int_math::div_exp2_floor(top_right_child_idx, 1);
  const Index2D top_right_parent_idx_unwrapped =
      int_math::div_exp2_floor(top_right_child_idx_unwrapped, 1);
  const IndexElement parent_height_idx = min_level_up;

  // Check if the nodes at min_level_up are direct neighbors
  if (bottom_left_child_idx + Index2D::Ones() ==
      top_right_child_idx_unwrapped) {
    // Check if they even share the same parent (node at min_level_up + 1)
    if (bottom_left_parent_idx == top_right_parent_idx_unwrapped) {
      // Since they do, checking the nodes at min_level_up is equivalent to
      // checking their common parent, so we do that instead as its cheaper
      return {lower_bound_levels_[parent_height_idx][bottom_left_parent_idx],
              upper_bound_levels_[parent_height_idx][bottom_left_parent_idx]};
    } else if (bottom_left_parent_idx.x() ==
                   top_right_parent_idx_unwrapped.x() ||
               bottom_left_parent_idx.y() ==
                   top_right_parent_idx_unwrapped.y()) {
      return {
          std::min(
              lower_bound_levels_[parent_height_idx][bottom_left_parent_idx],
              lower_bound_levels_[parent_height_idx][top_right_parent_idx]),
          std::max(
              upper_bound_levels_[parent_height_idx][bottom_left_parent_idx],
              upper_bound_levels_[parent_height_idx][top_right_parent_idx])};
    } else {
      // Check all four nodes at min_level_up
      if (min_level_up == 0) {
        return {
            std::min(
                {valueOrInit(range_image_->operator[](bottom_left_image_idx),
                             kUnknownRangeImageValueLowerBound),
                 valueOrInit(
                     range_image_->operator[](
                         {bottom_left_image_idx.x(), top_right_image_idx.y()}),
                     kUnknownRangeImageValueLowerBound),
                 valueOrInit(
                     range_image_->operator[](
                         {top_right_image_idx.x(), bottom_left_image_idx.y()}),
                     kUnknownRangeImageValueLowerBound),
                 valueOrInit(range_image_->operator[](top_right_image_idx),
                             kUnknownRangeImageValueLowerBound)}),
            std::max(
                {valueOrInit(range_image_->operator[](bottom_left_image_idx),
                             kUnknownRangeImageValueUpperBound),
                 valueOrInit(
                     range_image_->operator[](
                         {bottom_left_image_idx.x(), top_right_image_idx.y()}),
                     kUnknownRangeImageValueUpperBound),
                 valueOrInit(
                     range_image_->operator[](
                         {top_right_image_idx.x(), bottom_left_image_idx.y()}),
                     kUnknownRangeImageValueUpperBound),
                 valueOrInit(range_image_->operator[](top_right_image_idx),
                             kUnknownRangeImageValueUpperBound)})};
      } else {
        return {
            std::min(
                {lower_bound_levels_[child_height_idx][bottom_left_child_idx],
                 lower_bound_levels_[child_height_idx][{
                     bottom_left_child_idx.x(), top_right_child_idx.y()}],
                 lower_bound_levels_[child_height_idx][{
                     top_right_child_idx.x(), bottom_left_child_idx.y()}],
                 lower_bound_levels_[child_height_idx][top_right_child_idx]}),
            std::max(
                {upper_bound_levels_[child_height_idx][bottom_left_child_idx],
                 upper_bound_levels_[child_height_idx][{
                     bottom_left_child_idx.x(), top_right_child_idx.y()}],
                 upper_bound_levels_[child_height_idx][{
                     top_right_child_idx.x(), bottom_left_child_idx.y()}],
                 upper_bound_levels_[child_height_idx][top_right_child_idx]})};
      }
    }
  } else {
    // Since the nodes at min_level_up are not direct neighbors we need to go
    // one level up and check all four parents there
    return {
        std::min(
            {lower_bound_levels_[parent_height_idx][bottom_left_parent_idx],
             lower_bound_levels_[parent_height_idx][{bottom_left_parent_idx.x(),
                                                     top_right_parent_idx.y()}],
             lower_bound_levels_[parent_height_idx][{
                 top_right_parent_idx.x(), bottom_left_parent_idx.y()}],
             lower_bound_levels_[parent_height_idx][top_right_parent_idx]}),
        std::max(
            {upper_bound_levels_[parent_height_idx][bottom_left_parent_idx],
             upper_bound_levels_[parent_height_idx][{bottom_left_parent_idx.x(),
                                                     top_right_parent_idx.y()}],
             upper_bound_levels_[parent_height_idx][{
                 top_right_parent_idx.x(), bottom_left_parent_idx.y()}],
             upper_bound_levels_[parent_height_idx][top_right_parent_idx]})};
  }
}

template <bool azimuth_wraps_pi>
IntersectionType
HierarchicalRangeBounds2D<azimuth_wraps_pi>::getIntersectionType(
    const Index2D& bottom_left_image_idx, const Index2D& top_right_image_idx,
    FloatingPoint range_min, FloatingPoint range_max) const {
  const Bounds distance_bounds =
      getBounds(bottom_left_image_idx, top_right_image_idx);
  if (distance_bounds.upper < range_min) {
    return IntersectionType::kFullyUnknown;
  } else if (range_max < distance_bounds.lower) {
    return IntersectionType::kFreeOrUnknown;
  } else {
    return IntersectionType::kPossiblyOccupied;
  }
}

template <bool azimuth_wraps_pi>
template <typename BinaryFunctor>
std::vector<RangeImage2D>
HierarchicalRangeBounds2D<azimuth_wraps_pi>::computeReducedPyramid(
    const RangeImage2D& range_image, BinaryFunctor reduction_functor,
    FloatingPoint init) {
  CHECK(!azimuth_wraps_pi || bit_manip::popcount(range_image.getNumColumns()))
      << "For LiDAR range images that wrap around horizontally (FoV of "
         "360deg), only column numbers that are exact powers of 2 are "
         "currently supported.";

  const Index2D range_image_dims = range_image.getDimensions();
  const Index2D range_image_dims_scaled = {
      std::get<0>(scale_) * range_image_dims.x(),
      std::get<1>(scale_) * range_image_dims.y()};
  const int max_num_halvings =
      int_math::log2_ceil(range_image_dims_scaled.maxCoeff());

  std::vector<RangeImage2D> pyramid;
  pyramid.reserve(max_num_halvings);
  for (int level_idx = 0; level_idx < max_num_halvings; ++level_idx) {
    // Initialize the current level
    const Index2D level_dims =
        int_math::div_exp2_ceil(range_image_dims_scaled, level_idx + 1);
    RangeImage2D& current_level =
        pyramid.template emplace_back(level_dims.x(), level_dims.y(), init);

    // Reduce from the previous level or from the range image (for level 0)
    const RangeImage2D& previous_level =
        level_idx == 0 ? range_image : pyramid[level_idx - 1];
    const Index2D previous_level_dims = previous_level.getDimensions();

    for (const Index2D& idx :
         Grid<2>(Index2D::Zero(), level_dims - Index2D::Ones())) {
      Index2D min_child_idx = 2 * idx;
      Index2D max_child_idx = min_child_idx + Index2D::Ones();
      if (level_idx == 0) {
        min_child_idx = {min_child_idx.x() / std::get<0>(scale_),
                         min_child_idx.y() / std::get<1>(scale_)};
        max_child_idx = {max_child_idx.x() / std::get<0>(scale_),
                         max_child_idx.y() / std::get<1>(scale_)};
      }

      // Reduce the values in the 2x2 block of the previous level from
      // min_child_idx to max_child_idx while avoiding out-of-bounds access if
      // we're on the border. Where out-of-bounds access would occur, we
      // virtually pad the previous level with initial value 'init'.
      if ((min_child_idx.array() < previous_level_dims.array()).all()) {
        const FloatingPoint r00 =
            valueOrInit(previous_level[{min_child_idx.x(), min_child_idx.y()}],
                        init, level_idx);
        if (max_child_idx.x() < previous_level_dims.x()) {
          const FloatingPoint r10 = valueOrInit(
              previous_level[{max_child_idx.x(), min_child_idx.y()}], init,
              level_idx);
          const FloatingPoint first_col_reduced = reduction_functor(r00, r10);
          if (max_child_idx.y() < previous_level_dims.y()) {
            const FloatingPoint r01 = valueOrInit(
                previous_level[{min_child_idx.x(), max_child_idx.y()}], init,
                level_idx);
            const FloatingPoint r11 = valueOrInit(
                previous_level[{max_child_idx.x(), max_child_idx.y()}], init,
                level_idx);
            const FloatingPoint second_col_reduced =
                reduction_functor(r01, r11);
            current_level[idx] =
                reduction_functor(first_col_reduced, second_col_reduced);
          } else if (azimuth_wraps_pi &&
                     max_child_idx.y() <= previous_level_dims.y()) {
            const FloatingPoint r01 = valueOrInit(
                previous_level[{min_child_idx.x(), 0}], init, level_idx);
            const FloatingPoint r11 = valueOrInit(
                previous_level[{max_child_idx.x(), 0}], init, level_idx);
            const FloatingPoint second_col_reduced =
                reduction_functor(r01, r11);
            current_level[idx] =
                reduction_functor(first_col_reduced, second_col_reduced);
          } else {
            current_level[idx] = reduction_functor(first_col_reduced, init);
          }
        } else if (max_child_idx.y() < previous_level_dims.y()) {
          const FloatingPoint r01 = valueOrInit(
              previous_level[{min_child_idx.x(), max_child_idx.y()}], init,
              level_idx);
          const FloatingPoint first_row_reduced = reduction_functor(r00, r01);
          current_level[idx] = reduction_functor(first_row_reduced, init);
        } else if (azimuth_wraps_pi &&
                   max_child_idx.y() <= previous_level_dims.y()) {
          const FloatingPoint r01 = valueOrInit(
              previous_level[{min_child_idx.x(), 0}], init, level_idx);
          const FloatingPoint first_row_reduced = reduction_functor(r00, r01);
          current_level[idx] = reduction_functor(first_row_reduced, init);
        } else {
          current_level[idx] = reduction_functor(r00, init);
        }
      } else {
        current_level[idx] = init;
      }
    }
  }
  return pyramid;
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_BOUNDS_2D_INL_H_
