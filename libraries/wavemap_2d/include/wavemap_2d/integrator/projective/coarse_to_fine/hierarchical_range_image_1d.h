#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_1D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_1D_H_

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <wavemap_common/utils/int_math.h>

#include "wavemap_2d/integrator/projective/range_image_1d.h"

namespace wavemap {
struct Bounds {
  FloatingPoint lower;
  FloatingPoint upper;
};

class HierarchicalRangeImage1D {
 public:
  explicit HierarchicalRangeImage1D(std::shared_ptr<RangeImage1D> range_image)
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

  Bounds getRangeBounds(IndexElement left_idx, IndexElement right_idx) const;
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
  std::shared_ptr<RangeImage1D> range_image_;
  const std::vector<RangeImage1D::Data> lower_bounds_;
  const std::vector<RangeImage1D::Data> upper_bounds_;
  const BinaryTreeIndex::Element max_height_;

  template <typename BinaryFunctor>
  static std::vector<RangeImage1D::Data> computeReducedPyramid(
      const RangeImage1D& range_image, BinaryFunctor reduction_functor);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/hierarchical_range_image_1d_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_1D_H_
