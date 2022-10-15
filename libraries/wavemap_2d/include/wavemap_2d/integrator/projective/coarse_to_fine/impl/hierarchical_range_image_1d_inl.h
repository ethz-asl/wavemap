#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_1D_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_1D_INL_H_

#include <algorithm>
#include <vector>

namespace wavemap {
inline Bounds<FloatingPoint> HierarchicalRangeImage1D::getBounds(
    const BinaryTreeIndex& index) const {
  DCHECK_GE(index.height, 0);
  DCHECK_LE(index.height, max_height_);
  if (index.height == 0) {
    const FloatingPoint range_image_value =
        range_image_->getRange(index.position.x());
    return {range_image_value, range_image_value};
  } else {
    return {lower_bounds_[index.height - 1].getRange(index.position.x()),
            upper_bounds_[index.height - 1].getRange(index.position.x())};
  }
}

inline Bounds<FloatingPoint> HierarchicalRangeImage1D::getRangeBounds(
    IndexElement left_idx, IndexElement right_idx) const {
  DCHECK_LE(left_idx, right_idx);
  if (left_idx == right_idx) {
    const FloatingPoint range_image_value = range_image_->getRange(left_idx);
    return {range_image_value, range_image_value};
  }

  const IndexElement min_level_up = int_math::log2_floor(right_idx - left_idx);
  const IndexElement left_idx_shifted =
      int_math::div_exp2_floor(left_idx, min_level_up);
  const IndexElement right_idx_shifted =
      int_math::div_exp2_floor(right_idx, min_level_up);

  // Check if the nodes at min_level_up are direct neighbors
  if (left_idx_shifted + 1 == right_idx_shifted) {
    // Check if they even share the same parent (node at min_level_up + 1)
    if ((left_idx_shifted >> 1) == (right_idx_shifted >> 1)) {
      // Since they do, checking both nodes at min_level_up is equivalent to
      // checking their common parent, so we do that instead as its cheaper
      const IndexElement parent_height = min_level_up;
      const IndexElement parent_idx = left_idx_shifted >> 1;
      return {lower_bounds_[parent_height].getRange(parent_idx),
              upper_bounds_[parent_height].getRange(parent_idx)};
    } else {
      // Check both nodes at min_level_up
      const IndexElement height = min_level_up - 1;
      const IndexElement left_node_idx = left_idx_shifted;
      const IndexElement right_node_idx = right_idx_shifted;
      if (min_level_up == 0) {
        return {std::min(range_image_->getRange(left_node_idx),
                         range_image_->getRange(right_node_idx)),
                std::max(range_image_->getRange(left_node_idx),
                         range_image_->getRange(right_node_idx))};
      } else {
        return {std::min(lower_bounds_[height].getRange(left_node_idx),
                         lower_bounds_[height].getRange(right_node_idx)),
                std::max(upper_bounds_[height].getRange(left_node_idx),
                         upper_bounds_[height].getRange(right_node_idx))};
      }
    }
  } else {
    // Since the nodes at min_level_up are not direct neighbors we need to go
    // one level up and check both parents there
    DCHECK(left_idx_shifted + 2 == right_idx_shifted);
    const NdtreeIndexElement parent_height = min_level_up;
    const IndexElement left_parent_idx = left_idx_shifted >> 1;
    const IndexElement right_parent_idx = right_idx_shifted >> 1;
    return {std::min(lower_bounds_[parent_height].getRange(left_parent_idx),
                     lower_bounds_[parent_height].getRange(right_parent_idx)),
            std::max(upper_bounds_[parent_height].getRange(left_parent_idx),
                     upper_bounds_[parent_height].getRange(right_parent_idx))};
  }
}

template <typename BinaryFunctor>
std::vector<RangeImage1D> HierarchicalRangeImage1D::computeReducedPyramid(
    const RangeImage1D& range_image, BinaryFunctor reduction_functor,
    FloatingPoint init) {
  const int range_image_width = range_image.getNumBeams();
  const int max_num_halvings = int_math::log2_ceil(range_image_width);

  std::vector<RangeImage1D> pyramid;
  pyramid.reserve(max_num_halvings);
  for (int level_idx = 0; level_idx < max_num_halvings; ++level_idx) {
    // Initialize the current level
    const int level_width =
        int_math::div_exp2_ceil(range_image_width, level_idx + 1);
    RangeImage1D& current_level =
        pyramid.template emplace_back(level_width, init);

    // Reduce from the previous level or from the range image (for level 0)
    const RangeImage1D& previous_level =
        level_idx == 0 ? range_image : pyramid[level_idx - 1];

    const bool previous_level_width_is_uneven =
        previous_level.getNumBeams() & 1;
    const int last_index = level_width - 1;
    const int last_regular_index = last_index - previous_level_width_is_uneven;
    for (int idx = 0; idx <= last_regular_index; ++idx) {
      const int first_child_idx = 2 * idx;
      const int second_child_idx = first_child_idx + 1;
      current_level.getRange(idx) =
          reduction_functor(previous_level.getRange(first_child_idx),
                            previous_level.getRange(second_child_idx));
    }
    if (previous_level_width_is_uneven) {
      const int first_child_idx = 2 * last_index;
      current_level.getRange(last_index) =
          reduction_functor(previous_level.getRange(first_child_idx), init);
    }
  }
  return pyramid;
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_IMAGE_1D_INL_H_
