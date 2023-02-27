#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "wavemap/data_structure/image.h"
#include "wavemap/integrator/projective/update_type.h"

namespace wavemap {
class HierarchicalRangeBounds {
 public:
  explicit HierarchicalRangeBounds(Image<>::ConstPtr range_image,
                                   bool azimuth_wraps_pi,
                                   FloatingPoint min_range)
      : range_image_(std::move(range_image)),
        azimuth_wraps_pi_(azimuth_wraps_pi),
        min_range_(min_range) {
    DCHECK_EQ(lower_bound_levels_.size(), max_height_);
    DCHECK_EQ(upper_bound_levels_.size(), max_height_);
  }

  NdtreeIndexElement getMaxHeight() const { return max_height_; }
  FloatingPoint getMinRange() const { return min_range_; }
  static FloatingPoint getUnknownValueLowerBound() {
    return kUnknownValueLowerBound;
  }
  static FloatingPoint getUnknownValueUpperBound() {
    return kUnknownValueUpperBound;
  }
  static Index2D getImageToPyramidScaleFactor() {
    return {std::get<0>(scale_), std::get<1>(scale_)};
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
  bool hasUnobserved(const QuadtreeIndex& index) const;

  Bounds<FloatingPoint> getBounds(const Index2D& min_image_idx,
                                  const Index2D& max_image_idx) const;
  FloatingPoint getLowerBound(const Index2D& min_image_idx,
                              const Index2D& max_image_idx) const {
    // NOTE: We reuse getBounds() and trust the compiler to optimize out the
    // computation of unused (return) values during inlining.
    return getBounds(min_image_idx, max_image_idx).lower;
  }
  FloatingPoint getUpperBound(const Index2D& min_image_idx,
                              const Index2D& max_image_idx) const {
    // NOTE: We reuse getBounds() and trust the compiler to optimize out the
    // computation of unused (return) values during inlining.
    return getBounds(min_image_idx, max_image_idx).upper;
  }
  bool hasUnobserved(const Index2D& min_image_idx,
                     const Index2D& max_image_idx) const;

  UpdateType getUpdateType(const Index2D& min_image_idx,
                           const Index2D& max_image_idx,
                           FloatingPoint range_min,
                           FloatingPoint range_max) const;

 private:
  const Image<>::ConstPtr range_image_;
  const bool azimuth_wraps_pi_;

  // Below min_range, range image values are treated as unknown and set to init
  const FloatingPoint min_range_;

  static constexpr FloatingPoint kUnknownValueLowerBound = 1e4f;
  static constexpr FloatingPoint kUnknownValueUpperBound = 0.f;

  const std::vector<Image<>> lower_bound_levels_ = computeReducedPyramid(
      range_image_->transform(
          [=](auto val) { return valueOrInit(val, kUnknownValueLowerBound); }),
      [](auto a, auto b) { return std::min(a, b); }, kUnknownValueLowerBound);
  const std::vector<Image<>> upper_bound_levels_ = computeReducedPyramid(
      range_image_->transform(
          [=](auto val) { return valueOrInit(val, kUnknownValueUpperBound); }),
      [](auto a, auto b) { return std::max(a, b); }, kUnknownValueUpperBound);
  const std::vector<Image<bool>> unobserved_mask_levels_ =
      computeReducedPyramid(
          range_image_->transform([=](auto val) { return isUnobserved(val); }),
          [](auto a, auto b) { return a || b; }, true);

  const NdtreeIndexElement max_height_ =
      static_cast<NdtreeIndexElement>(lower_bound_levels_.size());
  // TODO(victorr): Make this configurable (or adjust it automatically)
  static constexpr std::tuple<IndexElement, IndexElement> scale_ = {2, 1};

  template <typename T = FloatingPoint, typename BinaryFunctor>
  std::vector<Image<T>> computeReducedPyramid(const Image<T>& range_image,
                                              BinaryFunctor reduction_functor,
                                              T init);

  bool isUnobserved(FloatingPoint value) const { return value < min_range_; }
  FloatingPoint valueOrInit(FloatingPoint value, FloatingPoint init) const {
    // NOTE: Point clouds often contain points near the sensor, for example
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
    if (isUnobserved(value)) {
      return init;
    }
    return value;
  }
};
}  // namespace wavemap

#include "wavemap/integrator/projective/coarse_to_fine/impl/hierarchical_range_bounds_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_H_
