#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_2D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_2D_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <wavemap_common/integrator/projective/intersection_type.h>

#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
template <bool azimuth_wraps_pi>
class HierarchicalRangeBounds2D {
 public:
  explicit HierarchicalRangeBounds2D(std::shared_ptr<RangeImage2D> range_image)
      : range_image_(std::move(range_image)),
        lower_bound_levels_(computeReducedPyramid(
            *range_image_, [](auto a, auto b) { return std::min(a, b); },
            kUnknownRangeImageValueLowerBound)),
        upper_bound_levels_(computeReducedPyramid(
            *range_image_, [](auto a, auto b) { return std::max(a, b); },
            kUnknownRangeImageValueUpperBound)),
        max_height_(
            static_cast<NdtreeIndexElement>(lower_bound_levels_.size())) {
    DCHECK_EQ(lower_bound_levels_.size(), max_height_);
    DCHECK_EQ(upper_bound_levels_.size(), max_height_);
  }

  NdtreeIndexElement getMaxHeight() const { return max_height_; }
  static FloatingPoint getUnknownRangeImageValueLowerBound() {
    return kUnknownRangeImageValueLowerBound;
  }
  static FloatingPoint getUnknownRangeImageValueUpperBound() {
    return kUnknownRangeImageValueUpperBound;
  }
  static FloatingPoint getRangeMin() { return kRangeMin; }

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

  Bounds<FloatingPoint> getBounds(const Index2D& bottom_left_image_idx,
                                  const Index2D& top_right_image_idx) const;
  FloatingPoint getLowerBound(const Index2D& bottom_left_image_idx,
                              const Index2D& top_right_image_idx) const {
    // NOTE: We reuse getRangeBoundsApprox() and trust the compiler to optimize
    //       out the computation of unused (return) values during inlining.
    return getBounds(bottom_left_image_idx, top_right_image_idx).lower;
  }
  FloatingPoint getUpperBound(const Index2D& bottom_left_image_idx,
                              const Index2D& top_right_image_idx) const {
    // NOTE: We reuse getRangeBoundsApprox() and trust the compiler to optimize
    //       out the computation of unused (return) values during inlining.
    return getBounds(bottom_left_image_idx, top_right_image_idx).upper;
  }

  IntersectionType getIntersectionType(const Index2D& bottom_left_image_idx,
                                       const Index2D& top_right_image_idx,
                                       FloatingPoint range_min,
                                       FloatingPoint range_max) const;

 private:
  const std::shared_ptr<const RangeImage2D> range_image_;

  static constexpr FloatingPoint kUnknownRangeImageValueLowerBound =
      std::numeric_limits<FloatingPoint>::max();
  static constexpr FloatingPoint kUnknownRangeImageValueUpperBound = 0.f;
  const std::vector<RangeImage2D> lower_bound_levels_;
  const std::vector<RangeImage2D> upper_bound_levels_;

  const NdtreeIndexElement max_height_;
  static constexpr std::tuple<IndexElement, IndexElement> scale_ = {2, 1};

  template <typename BinaryFunctor>
  static std::vector<RangeImage2D> computeReducedPyramid(
      const RangeImage2D& range_image, BinaryFunctor reduction_functor,
      FloatingPoint init);

  // TODO(victorr): Make this configurable
  // Below kRangeMin, range image values are treated as unknown
  static constexpr FloatingPoint kRangeMin = 0.5f;
  static FloatingPoint valueOrInit(FloatingPoint value, FloatingPoint init,
                                   int level_idx) {
    // NOTE: Point clouds often contains points near the sensor, for example
    //       from missing returns being encoded as zeros, wires or cages around
    //       the sensor or more generally points hitting the robot or operator's
    //       body. Filtering out such spurious low values leads to big runtime
    //       improvements. This is due to the fact that the range image
    //       intersector, used by all coarse-to-fine integrators, relies on the
    //       hierarchical range image to quickly get conservative bounds on the
    //       range values in sub-intervals of the range image. The hierarchical
    //       range image gathers these bounds from upper and lower bound image
    //       pyramids. The lower bounds pyramid is generated with min-pooling,
    //       so low values quickly spread and end up making large intervals very
    //       conservative, which in turns results in very large parts of the
    //       observed volume to be marked as possibly occupied.
    if (level_idx == 0 && value < kRangeMin) {
      return init;
    }
    return value;
  }
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/coarse_to_fine/impl/hierarchical_range_bounds_2d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_2D_H_
