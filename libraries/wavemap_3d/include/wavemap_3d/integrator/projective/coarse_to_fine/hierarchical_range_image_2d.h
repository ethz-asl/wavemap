#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_2D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_2D_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
class HierarchicalRangeImage2D {
 public:
  explicit HierarchicalRangeImage2D(std::shared_ptr<RangeImage2D> range_image)
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

  Bounds<FloatingPoint> getBounds(const QuadtreeIndex& index) const;
  FloatingPoint getLowerBound(const QuadtreeIndex& index) const {
    // NOTE: We reuse getBounds() and trust the compiler to optimize out the
    //       computation of unused (return) values during inlining.
    return getBounds(index).lower;
  }
  FloatingPoint getUpperBound(const QuadtreeIndex& index) const {
    // NOTE: We reuse getBounds() and trust the compiler to optimize out the
    //       computation of unused (return) values during inlining.
    return getBounds(index).upper;
  }

  Bounds<FloatingPoint> getRangeBounds(const Index2D& left_idx,
                                       const Index2D& right_idx) const;
  FloatingPoint getRangeLowerBound(const Index2D& left_idx,
                                   const Index2D& right_idx) const {
    // NOTE: We reuse getRangeBoundsApprox() and trust the compiler to optimize
    //       out the computation of unused (return) values during inlining.
    return getRangeBounds(left_idx, right_idx).lower;
  }
  FloatingPoint getRangeUpperBound(const Index2D& left_idx,
                                   const Index2D& right_idx) const {
    // NOTE: We reuse getRangeBoundsApprox() and trust the compiler to optimize
    //       out the computation of unused (return) values during inlining.
    return getRangeBounds(left_idx, right_idx).upper;
  }

 private:
  const std::shared_ptr<const RangeImage2D> range_image_;

  static constexpr FloatingPoint kUnknownRangeImageValueLowerBound =
      std::numeric_limits<FloatingPoint>::max();
  static constexpr FloatingPoint kUnknownRangeImageValueUpperBound = 0.f;
  const std::vector<RangeImage2D> lower_bounds_;
  const std::vector<RangeImage2D> upper_bounds_;

  const NdtreeIndexElement max_height_;

  template <typename BinaryFunctor>
  static std::vector<RangeImage2D> computeReducedPyramid(
      const RangeImage2D& range_image, BinaryFunctor reduction_functor,
      FloatingPoint init);
  static FloatingPoint valueOrInit(FloatingPoint value, FloatingPoint init,
                                   int level_idx) {
    if (level_idx == 0 && value < kEpsilon) {
      return init;
    }
    return value;
  }
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/coarse_to_fine/impl/hierarchical_range_image_2d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_IMAGE_2D_H_
