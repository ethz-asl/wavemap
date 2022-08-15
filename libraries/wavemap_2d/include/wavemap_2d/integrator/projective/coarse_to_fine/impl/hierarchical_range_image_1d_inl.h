#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_1D_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_1D_INL_H_

#include <algorithm>
#include <vector>

namespace wavemap {
inline Bounds<FloatingPoint> HierarchicalRangeImage1D::getRangeBounds(
    IndexElement left_idx, IndexElement right_idx) const {
  DCHECK_LE(left_idx, right_idx);
  if (left_idx == right_idx) {
    const FloatingPoint range_image_value = range_image_->operator[](left_idx);
    return {range_image_value, range_image_value};
  }

  const IndexElement min_level_up = int_math::log2_floor(right_idx - left_idx);
  const IndexElement left_idx_shifted = left_idx >> min_level_up;
  const IndexElement right_idx_shifted = right_idx >> min_level_up;

  // Check if the nodes at min_level_up are direct neighbors
  if (left_idx_shifted + 1 == right_idx_shifted) {
    // Check if they even share the same parent (node at min_level_up + 1)
    if ((left_idx_shifted & 0b10) == (right_idx_shifted & 0b10)) {
      // Since they do, we only need to check the parent which is equivalent
      // to checking both nodes at min_level_up but cheaper
      const BinaryTreeIndex::Element parent_height = min_level_up;
      const BinaryTreeIndex::Element parent_idx = left_idx_shifted >> 1;
      return {lower_bounds_[parent_height][parent_idx],
              upper_bounds_[parent_height][parent_idx]};
    } else {
      // Check both nodes at min_level_up
      const BinaryTreeIndex::Element height = min_level_up - 1;
      const IndexElement left_node_idx = left_idx_shifted;
      const IndexElement right_node_idx = right_idx_shifted;
      if (min_level_up == 0) {
        return {std::min(range_image_->operator[](left_node_idx),
                         range_image_->operator[](right_node_idx)),
                std::max(range_image_->operator[](left_node_idx),
                         range_image_->operator[](right_node_idx))};
      } else {
        return {std::min(lower_bounds_[height][left_node_idx],
                         lower_bounds_[height][right_node_idx]),
                std::max(upper_bounds_[height][left_node_idx],
                         upper_bounds_[height][right_node_idx])};
      }
    }
  } else {
    // Since the nodes at min_level_up are not direct neighbors we need to go
    // one level up and check both parents there
    DCHECK(left_idx_shifted + 2 == right_idx_shifted);
    const BinaryTreeIndex::Element parent_height = min_level_up;
    const IndexElement left_parent_idx = left_idx_shifted >> 1;
    const IndexElement right_parent_idx = right_idx_shifted >> 1;
    return {std::min(lower_bounds_[parent_height][left_parent_idx],
                     lower_bounds_[parent_height][right_parent_idx]),
            std::max(upper_bounds_[parent_height][left_parent_idx],
                     upper_bounds_[parent_height][right_parent_idx])};
  }
}

template <typename BinaryFunctor>
std::vector<RangeImage1D::Data> HierarchicalRangeImage1D::computeReducedPyramid(
    const RangeImage1D& range_image, BinaryFunctor reduction_functor) {
  const int original_width = range_image.getNumBeams();
  const int max_num_halvings = int_math::log2_ceil(original_width);
  std::vector<RangeImage1D::Data> pyramid(max_num_halvings);

  const int last_reduction_level = max_num_halvings - 1;
  for (int level_idx = 0; level_idx <= last_reduction_level; ++level_idx) {
    // Zero initialize the current level
    RangeImage1D::Data& current_level = pyramid[level_idx];
    const int level_width = int_math::exp2(last_reduction_level - level_idx);
    current_level = RangeImage1D::Data::Zero(1, level_width);
    // Reduce
    if (level_idx == 0) {
      // For the first level, reduce from the original range image
      const int image_width = range_image.getNumBeams();
      const int half_image_width = image_width >> 1;  // Always rounded down
      for (int idx = 0; idx < half_image_width; ++idx) {
        const int first_child_idx = 2 * idx;
        const int second_child_idx = first_child_idx + 1;
        current_level[idx] = reduction_functor(range_image[first_child_idx],
                                               range_image[second_child_idx]);
      }
      const bool image_width_is_even = !(image_width & 0b1);
      if (!image_width_is_even) {
        const int first_child_idx = 2 * half_image_width;
        current_level[half_image_width] = range_image[first_child_idx];
      }
    } else {
      // Continue reducing from the previous reduction level otherwise
      const RangeImage1D::Data& previous_level = pyramid[level_idx - 1];
      for (int idx = 0; idx < level_width; ++idx) {
        const int first_child_idx = 2 * idx;
        const int second_child_idx = first_child_idx + 1;
        current_level[idx] = reduction_functor(
            previous_level[first_child_idx], previous_level[second_child_idx]);
      }
    }
  }
  return pyramid;
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_1D_INL_H_
