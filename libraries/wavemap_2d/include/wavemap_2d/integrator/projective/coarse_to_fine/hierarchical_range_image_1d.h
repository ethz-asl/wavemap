#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_1D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_1D_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <wavemap_common/utils/int_math.h>

#include "wavemap_2d/integrator/projective/range_image_1d.h"

namespace wavemap {
class HierarchicalRangeImage1D {
 public:
  explicit HierarchicalRangeImage1D(std::shared_ptr<RangeImage1D> range_image)
      : range_image_(std::move(range_image)),
        lower_bounds_(computeReducedPyramid(
            *range_image_, [](auto a, auto b) { return std::min(a, b); },
            kUnknownRangeImageValueLowerBound)),
        upper_bounds_(computeReducedPyramid(
            *range_image_, [](auto a, auto b) { return std::max(a, b); },
            kUnknownRangeImageValueUpperBound)),
        max_height_(static_cast<NdtreeIndexElement>(lower_bounds_.size())) {
    DCHECK_EQ(lower_bounds_.size(), max_height_);
    DCHECK_EQ(upper_bounds_.size(), max_height_);
  }

  NdtreeIndexElement getMaxHeight() const { return max_height_; }
  static FloatingPoint getUnknownRangeImageValueLowerBound() {
    return kUnknownRangeImageValueLowerBound;
  }
  static FloatingPoint getUnknownRangeImageValueUpperBound() {
    return kUnknownRangeImageValueUpperBound;
  }

  Bounds<FloatingPoint> getBounds(const BinaryTreeIndex& index) const;
  FloatingPoint getLowerBound(const BinaryTreeIndex& index) const {
    // NOTE: We reuse getBounds() and trust the compiler to optimize out the
    //       computation of unused (return) values during inlining.
    return getBounds(index).lower;
  }
  FloatingPoint getUpperBound(const BinaryTreeIndex& index) const {
    // NOTE: We reuse getBounds() and trust the compiler to optimize out the
    //       computation of unused (return) values during inlining.
    return getBounds(index).upper;
  }

  Bounds<FloatingPoint> getRangeBounds(IndexElement left_idx,
                                       IndexElement right_idx) const;
  FloatingPoint getRangeLowerBound(IndexElement left_idx,
                                   IndexElement right_idx) const {
    // NOTE: We reuse getRangeBounds() and trust the compiler to optimize
    //       out the computation of unused (return) values during inlining.
    return getRangeBounds(left_idx, right_idx).lower;
  }
  FloatingPoint getRangeUpperBound(IndexElement left_idx,
                                   IndexElement right_idx) const {
    // NOTE: We reuse getRangeBounds() and trust the compiler to optimize
    //       out the computation of unused (return) values during inlining.
    return getRangeBounds(left_idx, right_idx).upper;
  }

 private:
  const std::shared_ptr<const RangeImage1D> range_image_;

  static constexpr FloatingPoint kUnknownRangeImageValueLowerBound =
      std::numeric_limits<FloatingPoint>::max();
  static constexpr FloatingPoint kUnknownRangeImageValueUpperBound = 0.f;
  const std::vector<RangeImage1D> lower_bounds_;
  const std::vector<RangeImage1D> upper_bounds_;

  const NdtreeIndexElement max_height_;

  template <typename BinaryFunctor>
  static std::vector<RangeImage1D> computeReducedPyramid(
      const RangeImage1D& range_image, BinaryFunctor reduction_functor,
      FloatingPoint init);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/hierarchical_range_image_1d_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_1D_H_
