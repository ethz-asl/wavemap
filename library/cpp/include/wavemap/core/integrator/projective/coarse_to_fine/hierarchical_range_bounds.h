#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "wavemap/core/data_structure/image.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/integrator/projection_model/projector_base.h"
#include "wavemap/core/integrator/projective/update_type.h"

namespace wavemap {
class HierarchicalRangeBounds {
 public:
  explicit HierarchicalRangeBounds(
      Image<>::ConstPtr range_image, bool azimuth_wraps_pi,
      FloatingPoint min_range, const ProjectorBase* projection_model = nullptr)
      : range_image_(std::move(range_image)),
        min_range_(min_range),
        azimuth_wraps_pi_(azimuth_wraps_pi),
        image_to_pyramid_scale_factor_(
            computeImageToPyramidScaleFactor(projection_model)) {
    CHECK((image_to_pyramid_scale_factor_.array() == 1).any())
        << "For scale: " << image_to_pyramid_scale_factor_;
    CHECK((1 <= image_to_pyramid_scale_factor_.array()).all())
        << "For scale: " << image_to_pyramid_scale_factor_;
    CHECK((image_to_pyramid_scale_factor_.array() <= 2).all())
        << "For scale: " << image_to_pyramid_scale_factor_;
    DCHECK_EQ(lower_bound_levels_.size(), max_height_);
    DCHECK_EQ(upper_bound_levels_.size(), max_height_);
  }

  IndexElement getMaxHeight() const { return max_height_; }
  FloatingPoint getMinRange() const { return min_range_; }
  static FloatingPoint getUnknownValueLowerBound() {
    return kUnknownValueLowerBound;
  }
  static FloatingPoint getUnknownValueUpperBound() {
    return kUnknownValueUpperBound;
  }
  Index2D getImageToPyramidScaleFactor() const {
    return image_to_pyramid_scale_factor_;
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
  static constexpr FloatingPoint kUnknownValueLowerBound = 1e4f;
  static constexpr FloatingPoint kUnknownValueUpperBound = 0.f;

  const Image<>::ConstPtr range_image_;

  // Below min_range, range image values are treated as unknown and set to init
  const FloatingPoint min_range_;

  // Params defining how the image is mapped to the pyramid
  const bool azimuth_wraps_pi_;
  const Index2D image_to_pyramid_scale_factor_;
  static Index2D computeImageToPyramidScaleFactor(
      const ProjectorBase* projector = nullptr);

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

  const IndexElement max_height_ =
      static_cast<IndexElement>(lower_bound_levels_.size());

  template <typename T = FloatingPoint, typename BinaryFunctor>
  std::vector<Image<T>> computeReducedPyramid(const Image<T>& range_image,
                                              BinaryFunctor reduction_functor,
                                              T init);

  bool isUnobserved(FloatingPoint value) const { return value < min_range_; }
  FloatingPoint valueOrInit(FloatingPoint value, FloatingPoint init) const {
    // NOTE: Pointclouds often contain points near the sensor, for example
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

#include "wavemap/core/integrator/projective/coarse_to_fine/impl/hierarchical_range_bounds_inl.h"

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_BOUNDS_H_
