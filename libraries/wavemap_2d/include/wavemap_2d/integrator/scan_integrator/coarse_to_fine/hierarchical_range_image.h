#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_H_

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <wavemap_common/utils/int_math.h>

#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap {
struct Bounds {
  FloatingPoint lower;
  FloatingPoint upper;
};

class HierarchicalRangeImage {
 public:
  explicit HierarchicalRangeImage(std::shared_ptr<RangeImage> range_image)
      : range_image_(std::move(range_image)),
        lower_bounds_(computeReducedPyramid(
            *range_image_, [](auto a, auto b) { return std::min(a, b); })),
        upper_bounds_(computeReducedPyramid(
            *range_image_, [](auto a, auto b) { return std::max(a, b); })),
        max_height_(
            static_cast<BinaryTreeIndex::Element>(lower_bounds_.size())) {
    DCHECK_EQ(lower_bounds_.size(), max_height_);
    DCHECK_EQ(upper_bounds_.size(), max_height_);
  }

  BinaryTreeIndex::Element getMaxHeight() const { return max_height_; }
  BinaryTreeIndex::Element getNumBoundLevels() const { return max_height_ - 1; }

  Bounds getBounds(const BinaryTreeIndex& index) const {
    DCHECK_GE(index.height, 0);
    DCHECK_LE(index.height, max_height_);
    if (index.height == 0) {
      const FloatingPoint range_image_value =
          range_image_->operator[](index.position.x());
      return {range_image_value, range_image_value};
    } else {
      return {lower_bounds_[index.height - 1][index.position.x()],
              upper_bounds_[index.height - 1][index.position.x()]};
    }
  }
  FloatingPoint getLowerBound(const BinaryTreeIndex& index) const {
    return getBounds(index).lower;
  }
  FloatingPoint getUpperBound(const BinaryTreeIndex& index) const {
    return getBounds(index).upper;
  }
  // NOTE: We reuse getBounds() to get .upper/.lower and trust the compiler to
  //       optimize out unused (return) values during inlining.

  Bounds getRangeBounds(IndexElement left_idx, IndexElement right_idx) const {
    DCHECK_LE(left_idx, right_idx);
    if (left_idx == right_idx) {
      const FloatingPoint range_image_value =
          range_image_->operator[](left_idx);
      return {range_image_value, range_image_value};
    }

    const IndexElement min_level_up =
        int_math::log2_floor(right_idx - left_idx);
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
  FloatingPoint getRangeLowerBound(IndexElement left_idx,
                                   IndexElement right_idx) const {
    return getRangeBounds(left_idx, right_idx).lower;
  }
  FloatingPoint getRangeUpperBound(IndexElement left_idx,
                                   IndexElement right_idx) const {
    return getRangeBounds(left_idx, right_idx).upper;
  }
  // NOTE: We reuse getRangeBoundsApprox() to get .upper/.lower and trust the
  //       compiler to optimize out unused (return) values during inlining.

 private:
  std::shared_ptr<RangeImage> range_image_;
  const std::vector<RangeImage::RangeImageData> lower_bounds_;
  const std::vector<RangeImage::RangeImageData> upper_bounds_;
  const BinaryTreeIndex::Element max_height_;

  template <typename BinaryFunctor>
  static std::vector<RangeImage::RangeImageData> computeReducedPyramid(
      const RangeImage& range_image, BinaryFunctor reduction_functor) {
    const int original_width = range_image.getNumBeams();
    const int max_num_halvings = int_math::log2_ceil(original_width);
    std::vector<RangeImage::RangeImageData> pyramid(max_num_halvings);

    const int last_reduction_level = max_num_halvings - 1;
    for (int level_idx = 0; level_idx <= last_reduction_level; ++level_idx) {
      // Zero initialize the current level
      RangeImage::RangeImageData& current_level = pyramid[level_idx];
      const int level_width = int_math::exp2(last_reduction_level - level_idx);
      current_level = RangeImage::RangeImageData::Zero(1, level_width);
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
        const RangeImage::RangeImageData& previous_level =
            pyramid[level_idx - 1];
        for (int idx = 0; idx < level_width; ++idx) {
          const int first_child_idx = 2 * idx;
          const int second_child_idx = first_child_idx + 1;
          current_level[idx] =
              reduction_functor(previous_level[first_child_idx],
                                previous_level[second_child_idx]);
        }
      }
    }
    return pyramid;
  }
};
}  // namespace wavemap
#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_H_
