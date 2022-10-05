#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_2D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_2D_INL_H_

#include <algorithm>
#include <vector>

#include <wavemap_common/iterator/grid_iterator.h>
#include <wavemap_common/utils/bit_manipulation.h>

namespace wavemap {
template <bool azimuth_wraps_around_pi>
inline Bounds<FloatingPoint>
HierarchicalRangeImage2D<azimuth_wraps_around_pi>::getBounds(
    const QuadtreeIndex& index) const {
  DCHECK_GE(index.height, 0);
  DCHECK_LE(index.height, max_height_);
  if (index.height == 0) {
    const FloatingPoint range_image_value =
        range_image_->operator[](index.position);
    return {range_image_value, range_image_value};
  } else {
    return {lower_bounds_[index.height - 1][index.position],
            upper_bounds_[index.height - 1][index.position]};
  }
}

template <bool azimuth_wraps_around_pi>
inline Bounds<FloatingPoint>
HierarchicalRangeImage2D<azimuth_wraps_around_pi>::getRangeBounds(
    const Index2D& bottom_left_idx, const Index2D& top_right_idx) const {
  Index2D top_right_idx_unwrapped = top_right_idx;
  if (azimuth_wraps_around_pi &&
      top_right_idx_unwrapped.y() < bottom_left_idx.y()) {
    top_right_idx_unwrapped.y() += range_image_->getNumColumns();
  }
  DCHECK((bottom_left_idx.array() <= top_right_idx_unwrapped.array()).all());

  const IndexElement min_level_up = int_math::log2_floor(
      (top_right_idx_unwrapped - bottom_left_idx).maxCoeff());
  const Index2D bottom_left_idx_shifted =
      int_math::div_exp2_floor(bottom_left_idx, min_level_up);
  const Index2D top_right_idx_shifted =
      int_math::div_exp2_floor(top_right_idx, min_level_up);
  const Index2D top_right_idx_unwrapped_shifted =
      int_math::div_exp2_floor(top_right_idx_unwrapped, min_level_up);

  // Check if the nodes at min_level_up are direct neighbors
  if (bottom_left_idx_shifted + Index2D::Ones() ==
      top_right_idx_unwrapped_shifted) {
    // Check if they even share the same parent (node at min_level_up + 1)
    if (int_math::div_exp2_floor(bottom_left_idx_shifted, 1) ==
        int_math::div_exp2_floor(top_right_idx_unwrapped_shifted, 1)) {
      // Since they do, checking both nodes at min_level_up is equivalent to
      // checking their common parent, so we do that instead as its cheaper
      const IndexElement parent_height = min_level_up;
      const Index2D parent_idx =
          int_math::div_exp2_floor(bottom_left_idx_shifted, 1);
      return {lower_bounds_[parent_height][parent_idx],
              upper_bounds_[parent_height][parent_idx]};
    } else {
      // Check both nodes at min_level_up
      const IndexElement height = min_level_up - 1;
      const Index2D& left_node_idx = bottom_left_idx_shifted;
      const Index2D& right_node_idx = top_right_idx_shifted;
      if (min_level_up == 0) {
        const auto image_values = {
            range_image_->operator[](left_node_idx),
            range_image_->operator[]({left_node_idx.x(), right_node_idx.y()}),
            range_image_->operator[]({right_node_idx.x(), left_node_idx.y()}),
            range_image_->operator[](right_node_idx)};
        return {std::min(image_values), std::max(image_values)};
      } else {
        return {
            std::min(
                {lower_bounds_[height][left_node_idx],
                 lower_bounds_[height][{left_node_idx.x(), right_node_idx.y()}],
                 lower_bounds_[height][{right_node_idx.x(), left_node_idx.y()}],
                 lower_bounds_[height][right_node_idx]}),
            std::max(
                {upper_bounds_[height][left_node_idx],
                 upper_bounds_[height][{left_node_idx.x(), right_node_idx.y()}],
                 upper_bounds_[height][{right_node_idx.x(), left_node_idx.y()}],
                 upper_bounds_[height][right_node_idx]})};
      }
    }
  } else {
    // Since the nodes at min_level_up are not direct neighbors we need to go
    // one level up and check both parents there
    const IndexElement parent_height = min_level_up;
    const Index2D left_parent_idx =
        int_math::div_exp2_floor(bottom_left_idx_shifted, 1);
    const Index2D right_parent_idx =
        int_math::div_exp2_floor(top_right_idx_shifted, 1);
    return {
        std::min({lower_bounds_[parent_height][left_parent_idx],
                  lower_bounds_[parent_height]
                               [{left_parent_idx.x(), right_parent_idx.y()}],
                  lower_bounds_[parent_height]
                               [{right_parent_idx.x(), left_parent_idx.y()}],
                  lower_bounds_[parent_height][right_parent_idx]}),
        std::max({upper_bounds_[parent_height][left_parent_idx],
                  upper_bounds_[parent_height]
                               [{left_parent_idx.x(), right_parent_idx.y()}],
                  upper_bounds_[parent_height]
                               [{right_parent_idx.x(), left_parent_idx.y()}],
                  upper_bounds_[parent_height][right_parent_idx]})};
  }
}

template <bool azimuth_wraps_around_pi>
template <typename BinaryFunctor>
std::vector<RangeImage2D>
HierarchicalRangeImage2D<azimuth_wraps_around_pi>::computeReducedPyramid(
    const RangeImage2D& range_image, BinaryFunctor reduction_functor,
    FloatingPoint init) {
  CHECK(!azimuth_wraps_around_pi ||
        bit_manip::popcount(range_image.getNumColumns()))
      << "For LiDAR range images that wrap around horizontally (FoV of "
         "360deg), only column numbers that are exact powers of 2 are "
         "currently supported.";

  const Index2D range_image_dims = range_image.getDimensions();
  const int max_num_halvings = int_math::log2_ceil(range_image_dims.maxCoeff());

  std::vector<RangeImage2D> pyramid;
  pyramid.reserve(max_num_halvings);
  for (int level_idx = 0; level_idx < max_num_halvings; ++level_idx) {
    // Initialize the current level
    const Index2D level_dims =
        int_math::div_exp2_ceil(range_image_dims, level_idx + 1);
    RangeImage2D& current_level =
        pyramid.template emplace_back(level_dims.x(), level_dims.y(), init);

    // Reduce from the previous level or from the range image (for level 0)
    const RangeImage2D& previous_level =
        level_idx == 0 ? range_image : pyramid[level_idx - 1];
    const Index2D previous_level_dims = previous_level.getDimensions();

    for (const Index2D& idx :
         Grid<2>(Index2D::Zero(), level_dims - Index2D::Ones())) {
      const Index2D min_child_idx = 2 * idx;
      const Index2D max_child_idx = min_child_idx + Index2D::Ones();

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
          } else if (azimuth_wraps_around_pi &&
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
        } else if (azimuth_wraps_around_pi &&
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

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_2D_INL_H_
