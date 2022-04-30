#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_H_

#include <algorithm>
#include <numeric>
#include <vector>

#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap_2d {
struct Bounds {
  FloatingPoint lower;
  FloatingPoint upper;
};

class HierarchicalRangeImage {
 public:
  explicit HierarchicalRangeImage(const RangeImage& range_image)
      : range_image_(range_image),
        lower_bounds_(computeReducedPyramid(
            range_image_, [](auto a, auto b) { return std::min(a, b); })),
        upper_bounds_(computeReducedPyramid(
            range_image_, [](auto a, auto b) { return std::max(a, b); })) {
    DCHECK_EQ(lower_bounds_.size(), upper_bounds_.size());
  }

  FloatingPoint getLowerBound(const BinaryTreeIndex& index) const {
    DCHECK_GE(index.depth, 0);
    DCHECK_LE(index.depth, lower_bounds_.size());
    if (index.depth ==
        static_cast<BinaryTreeIndex::Element>(lower_bounds_.size())) {
      return range_image_[index.position.x()];
    } else {
      return lower_bounds_[index.depth][index.position.x()];
    }
  }
  FloatingPoint getUpperBound(const BinaryTreeIndex& index) const {
    DCHECK_GE(index.depth, 0);
    DCHECK_LE(index.depth, upper_bounds_.size());
    if (index.depth ==
        static_cast<BinaryTreeIndex::Element>(upper_bounds_.size())) {
      return range_image_[index.position.x()];
    } else {
      return upper_bounds_[index.depth][index.position.x()];
    }
  }
  Bounds getBounds(const BinaryTreeIndex& index) const {
    DCHECK_GE(index.depth, 0);
    DCHECK_LE(index.depth, lower_bounds_.size());
    if (index.depth ==
        static_cast<BinaryTreeIndex::Element>(lower_bounds_.size())) {
      const FloatingPoint range_image_value = range_image_[index.position.x()];
      return {.lower = range_image_value, .upper = range_image_value};
    } else {
      return {.lower = lower_bounds_[index.depth][index.position.x()],
              .upper = upper_bounds_[index.depth][index.position.x()]};
    }
  }
  size_t getMaxDepth() const { return lower_bounds_.size(); }
  size_t getPyramidDepth() const { return lower_bounds_.size() - 1; }

 private:
  const RangeImage& range_image_;
  const std::vector<RangeImage::RangeImageData> lower_bounds_;
  const std::vector<RangeImage::RangeImageData> upper_bounds_;

  template <typename BinaryFunctor>
  static std::vector<RangeImage::RangeImageData> computeReducedPyramid(
      const RangeImage& range_image, BinaryFunctor reduction_functor) {
    const int original_width = static_cast<int>(range_image.getNBeams());
    const int max_num_halvings = std::ceil(std::log2(original_width));
    std::vector<RangeImage::RangeImageData> pyramid(max_num_halvings);

    const int first_reduction_level = max_num_halvings - 1;
    for (int level_idx = first_reduction_level; 0 <= level_idx; --level_idx) {
      // Zero initialize the current level
      RangeImage::RangeImageData& current_level = pyramid[level_idx];
      const int level_width = 1 << level_idx;
      current_level = RangeImage::RangeImageData::Zero(1, level_width);
      // Reduce
      if (level_idx == first_reduction_level) {
        // For the first level, reduce from the original range image
        const int image_width = static_cast<int>(range_image.getNBeams());
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
            pyramid[level_idx + 1];
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
}  // namespace wavemap_2d
#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_H_
