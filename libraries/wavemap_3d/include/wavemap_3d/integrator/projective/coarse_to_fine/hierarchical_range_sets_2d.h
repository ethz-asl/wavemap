#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_SETS_2D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_SETS_2D_H_

#include <memory>
#include <utility>
#include <vector>

#include <wavemap_common/integrator/projective/intersection_type.h>

#include "wavemap_3d/integrator/projective/range_image_2d.h"

namespace wavemap {
template <bool azimuth_wraps_pi>
class HierarchicalRangeSets2D {
 public:
  explicit HierarchicalRangeSets2D(std::shared_ptr<RangeImage2D> range_image)
      : range_image_(std::move(range_image)),
        levels_(computeReducedLevels(*range_image_)),
        max_height_(static_cast<NdtreeIndexElement>(levels_.size())) {
    DCHECK_EQ(levels_.size(), max_height_);
  }

  NdtreeIndexElement getMaxHeight() const { return max_height_; }
  static FloatingPoint getRangeMin() { return kRangeMin; }

  IntersectionType getIntersectionType(const Index2D& bottom_left_image_idx,
                                       const Index2D& top_right_image_idx,
                                       FloatingPoint range_min,
                                       FloatingPoint range_max) const;

 private:
  using RangeCellIdx = uint8_t;
  using RangeCellSet = std::vector<RangeCellIdx>;
  using RangeCellSetImage =
      Eigen::Matrix<RangeCellSet, Eigen::Dynamic, Eigen::Dynamic>;

  const std::shared_ptr<const RangeImage2D> range_image_;

  std::vector<RangeCellSetImage> levels_;
  const NdtreeIndexElement max_height_;

  static constexpr FloatingPoint kRangeMin = 0.2f;
  static constexpr FloatingPoint kRangeResolutionAt1m = 0.03f;
  static constexpr RangeCellIdx rangeToRangeCellIdx(FloatingPoint range);
  static constexpr RangeCellIdx rangeToRangeCellIdxClamped(FloatingPoint range);
  static constexpr FloatingPoint rangeCellIdxToRange(
      RangeCellIdx range_cell_idx);
  static constexpr FloatingPoint kRangeMax = 100.f;

  static std::vector<RangeCellSetImage> computeReducedLevels(
      const RangeImage2D& range_image);

  IntersectionType getIntersectionType(
      RangeCellIdx range_cell_min_idx, RangeCellIdx range_cell_max_idx,
      std::initializer_list<std::reference_wrapper<const RangeCellSet>>
          range_cell_sets) const;
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/coarse_to_fine/impl/hierarchical_range_sets_2d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_HIERARCHICAL_RANGE_SETS_2D_H_
