#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_SETS_2D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_SETS_2D_INL_H_

#include <algorithm>
#include <limits>
#include <vector>

#include <wavemap_common/iterator/grid_iterator.h>
#include <wavemap_common/utils/bit_manipulation.h>

namespace wavemap {
template <bool azimuth_wraps_pi>
IntersectionType HierarchicalRangeSets2D<azimuth_wraps_pi>::getIntersectionType(
    const Index2D& bottom_left_image_idx, const Index2D& top_right_image_idx,
    FloatingPoint range_min, FloatingPoint range_max) const {
  if (bottom_left_image_idx == top_right_image_idx) {
    const auto range = range_image_->operator[](bottom_left_image_idx);
    if (valueOrInit(range, 0.f) < range_min) {
      return IntersectionType::kFullyUnknown;
    } else if (range_max < valueOrInit(range, 1e3f)) {
      return IntersectionType::kFreeOrUnknown;
    } else {
      return IntersectionType::kPossiblyOccupied;
    }
  }

  DCHECK_LE(range_min, range_max);
  if (kMaxRepresentableRange < range_min) {
    return IntersectionType::kFullyUnknown;
  }

  Index2D top_right_image_idx_unwrapped = top_right_image_idx;
  if (azimuth_wraps_pi && top_right_image_idx.y() < bottom_left_image_idx.y()) {
    top_right_image_idx_unwrapped.y() += range_image_->getNumColumns();
  }
  DCHECK(
      (bottom_left_image_idx.array() <= top_right_image_idx_unwrapped.array())
          .all());

  const Index2D bottom_left_image_idx_scaled = {
      std::get<0>(scale_) * bottom_left_image_idx.x(),
      std::get<1>(scale_) * bottom_left_image_idx.y()};
  const Index2D top_right_image_idx_scaled = {
      std::get<0>(scale_) * top_right_image_idx.x(),
      std::get<1>(scale_) * top_right_image_idx.y()};
  const Index2D top_right_image_idx_unwrapped_scaled = {
      std::get<0>(scale_) * top_right_image_idx_unwrapped.x(),
      std::get<1>(scale_) * top_right_image_idx_unwrapped.y()};

  const IndexElement max_idx_diff =
      (top_right_image_idx_unwrapped_scaled - bottom_left_image_idx_scaled)
          .maxCoeff();
  const IndexElement min_level_up =
      max_idx_diff == 0 ? 0 : int_math::log2_floor(max_idx_diff);

  // Compute the node indices at the minimum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D bottom_left_child_idx =
      int_math::div_exp2_floor(bottom_left_image_idx_scaled, min_level_up);
  const Index2D top_right_child_idx =
      int_math::div_exp2_floor(top_right_image_idx_scaled, min_level_up);
  const Index2D top_right_child_idx_unwrapped = int_math::div_exp2_floor(
      top_right_image_idx_unwrapped_scaled, min_level_up);
  const RangeCellIdx range_min_child_idx = int_math::div_exp2_floor(
      rangeToRangeCellIdxClamped(range_min), min_level_up);
  const RangeCellIdx range_max_child_idx = int_math::div_exp2_floor(
      rangeToRangeCellIdxClamped(range_max), min_level_up);
  const IndexElement child_height_idx = min_level_up - 1;

  // Compute the node indices at the maximum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D& bottom_left_parent_idx =
      int_math::div_exp2_floor(bottom_left_child_idx, 1);
  const Index2D top_right_parent_idx =
      int_math::div_exp2_floor(top_right_child_idx, 1);
  const Index2D top_right_parent_idx_unwrapped =
      int_math::div_exp2_floor(top_right_child_idx_unwrapped, 1);
  const RangeCellIdx range_min_parent_idx =
      int_math::div_exp2_floor(range_min_child_idx, 1);
  const RangeCellIdx range_max_parent_idx =
      int_math::div_exp2_floor(range_max_child_idx, 1);
  const IndexElement parent_height_idx = min_level_up;
  DCHECK_LT(parent_height_idx, max_height_);

  // Check if the nodes at min_level_up are direct neighbors
  if (bottom_left_child_idx + Index2D::Ones() ==
      top_right_child_idx_unwrapped) {
    // Check if they even share the same parent (node at min_level_up + 1)
    if (bottom_left_parent_idx == top_right_parent_idx_unwrapped) {
      // Since they do, checking the nodes at min_level_up is equivalent to
      // checking their common parent, so we do that instead as its cheaper
      return getIntersectionType(
          range_min_parent_idx, range_max_parent_idx,
          {levels_[parent_height_idx](bottom_left_parent_idx.x(),
                                      bottom_left_parent_idx.y())});
    } else if (bottom_left_parent_idx.x() ==
                   top_right_parent_idx_unwrapped.x() ||
               bottom_left_parent_idx.y() ==
                   top_right_parent_idx_unwrapped.y()) {
      return getIntersectionType(
          range_min_parent_idx, range_max_parent_idx,
          {levels_[parent_height_idx](bottom_left_parent_idx.x(),
                                      bottom_left_parent_idx.y()),
           levels_[parent_height_idx](top_right_parent_idx.x(),
                                      top_right_parent_idx.y())});
    } else {
      // Check all four nodes at min_level_up
      if (min_level_up == 0) {
        const auto image_values = {
            range_image_->operator[](bottom_left_image_idx),
            range_image_->operator[](
                {bottom_left_image_idx.x(), top_right_image_idx.y()}),
            range_image_->operator[](
                {top_right_image_idx.x(), bottom_left_image_idx.y()}),
            range_image_->operator[](top_right_image_idx)};
        if (std::all_of(image_values.begin(), image_values.end(),
                        [range_min](auto range) {
                          return valueOrInit(range, 0.f) < range_min;
                        })) {
          return IntersectionType::kFullyUnknown;
        } else if (std::all_of(image_values.begin(), image_values.end(),
                               [range_max](auto range) {
                                 return range_max < valueOrInit(range, 1e3f);
                               })) {
          return IntersectionType::kFreeOrUnknown;
        } else {
          return IntersectionType::kPossiblyOccupied;
        }
      } else {
        return getIntersectionType(
            range_min_child_idx, range_max_child_idx,
            {levels_[child_height_idx](bottom_left_child_idx.x(),
                                       bottom_left_child_idx.y()),
             levels_[child_height_idx](bottom_left_child_idx.x(),
                                       top_right_child_idx.y()),
             levels_[child_height_idx](top_right_child_idx.x(),
                                       bottom_left_child_idx.y()),
             levels_[child_height_idx](top_right_child_idx.x(),
                                       top_right_child_idx.y())});
      }
    }
  } else {
    // Since the nodes at min_level_up are not direct neighbors we need to go
    // one level up and check all four parents there
    return getIntersectionType(
        range_min_parent_idx, range_max_parent_idx,
        {levels_[parent_height_idx](bottom_left_parent_idx.x(),
                                    bottom_left_parent_idx.y()),
         levels_[parent_height_idx](bottom_left_parent_idx.x(),
                                    top_right_parent_idx.y()),
         levels_[parent_height_idx](top_right_parent_idx.x(),
                                    bottom_left_parent_idx.y()),
         levels_[parent_height_idx](top_right_parent_idx.x(),
                                    top_right_parent_idx.y())});
  }
}

template <bool azimuth_wraps_pi>
std::vector<
    typename HierarchicalRangeSets2D<azimuth_wraps_pi>::RangeCellSetImage>
HierarchicalRangeSets2D<azimuth_wraps_pi>::computeReducedLevels(
    const RangeImage2D& range_image) {
  CHECK(!azimuth_wraps_pi || bit_manip::popcount(range_image.getNumColumns()))
      << "For LiDAR range images that wrap around horizontally (FoV of "
         "360deg), only column numbers that are exact powers of 2 are "
         "currently supported.";

  const Index2D range_image_dims = range_image.getDimensions();
  const Index2D range_image_dims_scaled = {
      std::get<0>(scale_) * range_image_dims.x(),
      std::get<1>(scale_) * range_image_dims.y()};
  const int max_num_halvings =
      int_math::log2_ceil(range_image_dims_scaled.maxCoeff());

  // Reduce the range image max_num_halvings times
  std::vector<RangeCellSetImage> levels;
  levels.reserve(max_num_halvings);
  for (int level_idx = 0; level_idx < max_num_halvings; ++level_idx) {
    // Initialize the current level
    const Index2D level_dims =
        int_math::div_exp2_ceil(range_image_dims_scaled, level_idx + 1);
    RangeCellSetImage& current_level =
        levels.template emplace_back(level_dims.x(), level_dims.y());
    const Index2D previous_level_dims =
        level_idx == 0
            ? range_image_dims_scaled
            : int_math::div_exp2_ceil(range_image_dims_scaled, level_idx);

    // Iterate over all azimuth and elevation bins
    std::vector<RangeCellIdx> reduced_child_range_indices;
    for (const Index2D& idx :
         Grid<2>(Index2D::Zero(), level_dims - Index2D::Ones())) {
      reduced_child_range_indices.clear();

      const Index2D min_child_idx = 2 * idx;
      // Iterate over all children of the current bin
      for (int relative_child_idx = 0; relative_child_idx < 4;
           ++relative_child_idx) {
        Index2D child_idx = {
            min_child_idx.x() + (relative_child_idx & 0b1),
            min_child_idx.y() + (relative_child_idx >> 1 & 0b1)};
        // Skip child indices if they're out-of-bounds
        if (previous_level_dims.x() <= child_idx.x() ||
            previous_level_dims.y() < child_idx.y()) {
          continue;
        } else if (child_idx.y() == previous_level_dims.y()) {
          if (azimuth_wraps_pi) {
            child_idx.y() = 0;
          } else {
            continue;
          }
        }

        if (level_idx == 0) {
          // At the first level, we gather the range indices from the range
          // img
          const Index2D child_idx_rescaled = {
              child_idx.x() / std::get<0>(scale_),
              child_idx.y() / std::get<1>(scale_)};
          const FloatingPoint child_range =
              range_image.operator[](child_idx_rescaled);
          if (kMinRepresentableRange < child_range &&
              child_range < kMaxRepresentableRange) {
            const RangeCellIdx reduced_child_range_idx =
                rangeToRangeCellIdxClamped(child_range) >> 1;
            reduced_child_range_indices.template emplace_back(
                reduced_child_range_idx);
          }
        } else {
          // Otherwise we gather them from the previous level
          const RangeCellSetImage& previous_level = levels[level_idx - 1];
          const RangeCellSet& child_range_indices =
              previous_level(child_idx.x(), child_idx.y());
          // Copy and reduce the child range indices
          std::transform(child_range_indices.cbegin(),
                         child_range_indices.cend(),
                         std::back_inserter(reduced_child_range_indices),
                         [](auto range_idx) { return range_idx >> 1; });
        }
      }

      // Insert the reduced child range cell indices into the current cell,
      // in sorted order and without duplicates
      RangeCellSet& current_cell = current_level(idx.x(), idx.y());
      std::sort(reduced_child_range_indices.begin(),
                reduced_child_range_indices.end());
      std::unique_copy(reduced_child_range_indices.cbegin(),
                       reduced_child_range_indices.cend(),
                       std::back_inserter(current_cell));
      current_cell.shrink_to_fit();
    }
  }

  return levels;
}

template <bool azimuth_wraps_pi>
constexpr typename HierarchicalRangeSets2D<azimuth_wraps_pi>::RangeCellIdx
HierarchicalRangeSets2D<azimuth_wraps_pi>::rangeToRangeCellIdx(
    FloatingPoint range) {
  constexpr FloatingPoint kRangeResAt1mInv = 1.f / kRangeResolutionAt1m;
  constexpr FloatingPoint kRangeMinInv = 1.f / kMinRepresentableRange;
  return static_cast<RangeCellIdx>(kRangeResAt1mInv *
                                   std::log(kRangeMinInv * range));
}

template <bool azimuth_wraps_pi>
constexpr typename HierarchicalRangeSets2D<azimuth_wraps_pi>::RangeCellIdx
HierarchicalRangeSets2D<azimuth_wraps_pi>::rangeToRangeCellIdxClamped(
    FloatingPoint range) {
  if (range <= kMinRepresentableRange) {
    return 0.f;
  } else if (kMaxRepresentableRange <= range) {
    return std::numeric_limits<RangeCellIdx>::max();
  }
  return static_cast<RangeCellIdx>(rangeToRangeCellIdx(range));
}

template <bool azimuth_wraps_pi>
constexpr FloatingPoint
HierarchicalRangeSets2D<azimuth_wraps_pi>::rangeCellIdxToRange(
    HierarchicalRangeSets2D::RangeCellIdx range_cell_idx) {
  return kMinRepresentableRange *
         std::exp(kRangeResolutionAt1m *
                  static_cast<FloatingPoint>(range_cell_idx));
}

template <bool azimuth_wraps_pi>
IntersectionType HierarchicalRangeSets2D<azimuth_wraps_pi>::getIntersectionType(
    HierarchicalRangeSets2D::RangeCellIdx range_cell_min_idx,
    HierarchicalRangeSets2D::RangeCellIdx range_cell_max_idx,
    std::initializer_list<std::reference_wrapper<const RangeCellSet>>
        range_cell_sets) const {
  if (std::all_of(range_cell_sets.begin(), range_cell_sets.end(),
                  [range_cell_min_idx](const RangeCellSet& range_cell_set) {
                    return range_cell_set.empty() ||
                           range_cell_set.back() < range_cell_min_idx;
                  })) {
    return IntersectionType::kFullyUnknown;
  }

  if (std::all_of(range_cell_sets.begin(), range_cell_sets.end(),
                  [range_cell_max_idx](const RangeCellSet& range_cell_set) {
                    return range_cell_set.empty() ||
                           range_cell_max_idx < range_cell_set.front();
                  })) {
    return IntersectionType::kFreeOrUnknown;
  }

  for (const RangeCellSet& range_cell_set : range_cell_sets) {
    for (auto range_cell_idx : range_cell_set) {
      if (range_cell_min_idx <= range_cell_idx &&
          range_cell_idx <= range_cell_max_idx) {
        return IntersectionType::kPossiblyOccupied;
      } else if (range_cell_max_idx < range_cell_idx) {
        break;
      }
    }
  }

  return IntersectionType::kFreeOrUnknown;
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_SETS_2D_INL_H_
