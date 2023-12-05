#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_BOUNDS_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_BOUNDS_INL_H_

#include <algorithm>
#include <vector>

#include "wavemap/utils/bits/bit_operations.h"
#include "wavemap/utils/iterate/grid_iterator.h"

namespace wavemap {
inline Bounds<FloatingPoint> HierarchicalRangeBounds::getBounds(
    const QuadtreeIndex& index) const {
  DCHECK_GE(index.height, 0);
  DCHECK_LE(index.height, max_height_);
  if (index.height == 0) {
    const FloatingPoint range_image_value = range_image_->at(index.position);
    return {range_image_value, range_image_value};
  } else {
    return {lower_bound_levels_[index.height - 1].at(index.position),
            upper_bound_levels_[index.height - 1].at(index.position)};
  }
}

inline bool HierarchicalRangeBounds::hasUnobserved(
    const QuadtreeIndex& index) const {
  DCHECK_GE(index.height, 0);
  DCHECK_LE(index.height, max_height_);
  if (index.height == 0) {
    return isUnobserved(range_image_->at(index.position));
  } else {
    return unobserved_mask_levels_[index.height - 1].at(index.position);
  }
}

inline Bounds<FloatingPoint> HierarchicalRangeBounds::getBounds(
    const Index2D& min_image_idx, const Index2D& max_image_idx) const {
  if (min_image_idx == max_image_idx) {
    const auto range = range_image_->at(min_image_idx);
    return {valueOrInit(range, kUnknownValueLowerBound),
            valueOrInit(range, kUnknownValueUpperBound)};
  }

  Index2D max_image_idx_unwrapped = max_image_idx;
  if (azimuth_wraps_pi_ && max_image_idx.y() < min_image_idx.y()) {
    max_image_idx_unwrapped.y() += range_image_->getNumColumns();
  }
  DCHECK((min_image_idx.array() <= max_image_idx_unwrapped.array()).all());

  const Index2D min_image_idx_scaled =
      image_to_pyramid_scale_factor_.cwiseProduct(min_image_idx);
  const Index2D max_image_idx_scaled =
      image_to_pyramid_scale_factor_.cwiseProduct(max_image_idx);
  const Index2D max_image_idx_unwrapped_scaled =
      image_to_pyramid_scale_factor_.cwiseProduct(max_image_idx_unwrapped);

  const IndexElement max_idx_diff =
      (max_image_idx_unwrapped_scaled - min_image_idx_scaled).maxCoeff();
  const IndexElement min_level_up =
      max_idx_diff == 0 ? 0 : int_math::log2_floor(max_idx_diff);

  // Compute the node indices at the minimum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D min_child_idx =
      int_math::div_exp2_floor(min_image_idx_scaled, min_level_up);
  const Index2D max_child_idx =
      int_math::div_exp2_floor(max_image_idx_scaled, min_level_up);
  const Index2D max_child_idx_unwrapped =
      int_math::div_exp2_floor(max_image_idx_unwrapped_scaled, min_level_up);
  const IndexElement child_height_idx = min_level_up - 1;

  // Compute the node indices at the maximum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D& min_parent_idx = int_math::div_exp2_floor(min_child_idx, 1);
  const Index2D max_parent_idx = int_math::div_exp2_floor(max_child_idx, 1);
  const Index2D max_parent_idx_unwrapped =
      int_math::div_exp2_floor(max_child_idx_unwrapped, 1);
  const IndexElement parent_height_idx = min_level_up;

  // Check if the children (at min_level_up) are direct neighbors
  if (((max_child_idx_unwrapped - min_child_idx).array().abs() <= 1).all()) {
    DCHECK(min_child_idx != max_child_idx_unwrapped);
    if (min_child_idx.x() == max_child_idx_unwrapped.x() ||
        min_child_idx.y() == max_child_idx_unwrapped.y()) {
      // If children are on the same row or column, only need to check 2 cells
      if (min_level_up == 0) {
        return {std::min(valueOrInit(range_image_->at(min_image_idx),
                                     kUnknownValueLowerBound),
                         valueOrInit(range_image_->at(max_image_idx),
                                     kUnknownValueLowerBound)),
                std::max(valueOrInit(range_image_->at(min_image_idx),
                                     kUnknownValueUpperBound),
                         valueOrInit(range_image_->at(max_image_idx),
                                     kUnknownValueUpperBound))};
      } else {
        return {
            std::min(lower_bound_levels_[child_height_idx].at(min_child_idx),
                     lower_bound_levels_[child_height_idx].at(max_child_idx)),
            std::max(upper_bound_levels_[child_height_idx].at(min_child_idx),
                     upper_bound_levels_[child_height_idx].at(max_child_idx))};
      }
    } else if (min_parent_idx == max_parent_idx_unwrapped) {
      // If the children share the same parent (node at min_level_up + 1),
      // checking the four children is equivalent to checking their common
      // parent, so we do that instead as its cheaper
      return {lower_bound_levels_[parent_height_idx].at(min_parent_idx),
              upper_bound_levels_[parent_height_idx].at(min_parent_idx)};
    } else {
      // Check all four children at min_level_up
      if (min_level_up == 0) {
        return {std::min({valueOrInit(range_image_->at(min_image_idx),
                                      kUnknownValueLowerBound),
                          valueOrInit(range_image_->at({min_image_idx.x(),
                                                        max_image_idx.y()}),
                                      kUnknownValueLowerBound),
                          valueOrInit(range_image_->at({max_image_idx.x(),
                                                        min_image_idx.y()}),
                                      kUnknownValueLowerBound),
                          valueOrInit(range_image_->at(max_image_idx),
                                      kUnknownValueLowerBound)}),
                std::max({valueOrInit(range_image_->at(min_image_idx),
                                      kUnknownValueUpperBound),
                          valueOrInit(range_image_->at({min_image_idx.x(),
                                                        max_image_idx.y()}),
                                      kUnknownValueUpperBound),
                          valueOrInit(range_image_->at({max_image_idx.x(),
                                                        min_image_idx.y()}),
                                      kUnknownValueUpperBound),
                          valueOrInit(range_image_->at(max_image_idx),
                                      kUnknownValueUpperBound)})};
      } else {
        return {
            std::min({lower_bound_levels_[child_height_idx].at(min_child_idx),
                      lower_bound_levels_[child_height_idx].at(
                          {min_child_idx.x(), max_child_idx.y()}),
                      lower_bound_levels_[child_height_idx].at(
                          {max_child_idx.x(), min_child_idx.y()}),
                      lower_bound_levels_[child_height_idx].at(max_child_idx)}),
            std::max(
                {upper_bound_levels_[child_height_idx].at(min_child_idx),
                 upper_bound_levels_[child_height_idx].at(
                     {min_child_idx.x(), max_child_idx.y()}),
                 upper_bound_levels_[child_height_idx].at(
                     {max_child_idx.x(), min_child_idx.y()}),
                 upper_bound_levels_[child_height_idx].at(max_child_idx)})};
      }
    }
  } else {
    // Since the nodes at min_level_up are not direct neighbors we need to go
    // one level up and check all four parents there
    return {
        std::min({lower_bound_levels_[parent_height_idx].at(min_parent_idx),
                  lower_bound_levels_[parent_height_idx].at(
                      {min_parent_idx.x(), max_parent_idx.y()}),
                  lower_bound_levels_[parent_height_idx].at(
                      {max_parent_idx.x(), min_parent_idx.y()}),
                  lower_bound_levels_[parent_height_idx].at(max_parent_idx)}),
        std::max({upper_bound_levels_[parent_height_idx].at(min_parent_idx),
                  upper_bound_levels_[parent_height_idx].at(
                      {min_parent_idx.x(), max_parent_idx.y()}),
                  upper_bound_levels_[parent_height_idx].at(
                      {max_parent_idx.x(), min_parent_idx.y()}),
                  upper_bound_levels_[parent_height_idx].at(max_parent_idx)})};
  }
}

inline bool HierarchicalRangeBounds::hasUnobserved(
    const Index2D& min_image_idx, const Index2D& max_image_idx) const {
  if (min_image_idx == max_image_idx) {
    return isUnobserved(range_image_->at(min_image_idx));
  }

  Index2D max_image_idx_unwrapped = max_image_idx;
  if (azimuth_wraps_pi_ && max_image_idx.y() < min_image_idx.y()) {
    max_image_idx_unwrapped.y() += range_image_->getNumColumns();
  }
  DCHECK((min_image_idx.array() <= max_image_idx_unwrapped.array()).all());

  const Index2D min_image_idx_scaled =
      image_to_pyramid_scale_factor_.cwiseProduct(min_image_idx);
  const Index2D max_image_idx_scaled =
      image_to_pyramid_scale_factor_.cwiseProduct(max_image_idx);
  const Index2D max_image_idx_unwrapped_scaled =
      image_to_pyramid_scale_factor_.cwiseProduct(max_image_idx_unwrapped);

  const IndexElement max_idx_diff =
      (max_image_idx_unwrapped_scaled - min_image_idx_scaled).maxCoeff();
  const IndexElement min_level_up =
      max_idx_diff == 0 ? 0 : int_math::log2_floor(max_idx_diff);

  // Compute the node indices at the minimum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D min_child_idx =
      int_math::div_exp2_floor(min_image_idx_scaled, min_level_up);
  const Index2D max_child_idx =
      int_math::div_exp2_floor(max_image_idx_scaled, min_level_up);
  const Index2D max_child_idx_unwrapped =
      int_math::div_exp2_floor(max_image_idx_unwrapped_scaled, min_level_up);
  const IndexElement child_height_idx = min_level_up - 1;

  // Compute the node indices at the maximum level we have to go up to fully
  // cover the interval with 4 nodes or less
  const Index2D& min_parent_idx = int_math::div_exp2_floor(min_child_idx, 1);
  const Index2D max_parent_idx = int_math::div_exp2_floor(max_child_idx, 1);
  const Index2D max_parent_idx_unwrapped =
      int_math::div_exp2_floor(max_child_idx_unwrapped, 1);
  const IndexElement parent_height_idx = min_level_up;

  // Check if the children (at min_level_up) are direct neighbors
  if (((max_child_idx_unwrapped - min_child_idx).array().abs() <= 1).all()) {
    DCHECK(min_child_idx != max_child_idx_unwrapped);
    if (min_child_idx.x() == max_child_idx_unwrapped.x() ||
        min_child_idx.y() == max_child_idx_unwrapped.y()) {
      // If children are on the same row or column, only need to check 2
      // cells
      if (min_level_up == 0) {
        return isUnobserved(range_image_->at(min_image_idx)) ||
               isUnobserved(range_image_->at(max_image_idx));
      } else {
        return unobserved_mask_levels_[child_height_idx].at(min_child_idx) ||
               unobserved_mask_levels_[child_height_idx].at(max_child_idx);
      }
    } else if (min_parent_idx == max_parent_idx_unwrapped) {
      // If the children share the same parent (node at min_level_up + 1),
      // checking the four children is equivalent to checking their common
      // parent, so we do that instead as its cheaper
      return unobserved_mask_levels_[parent_height_idx].at(min_parent_idx);
    } else {
      // Check all four children at min_level_up
      if (min_level_up == 0) {
        return isUnobserved(range_image_->at(min_image_idx)) ||
               isUnobserved(
                   range_image_->at({min_image_idx.x(), max_image_idx.y()})) ||
               isUnobserved(
                   range_image_->at({max_image_idx.x(), min_image_idx.y()})) ||
               isUnobserved(range_image_->at(max_image_idx));
      } else {
        return unobserved_mask_levels_[child_height_idx].at(min_child_idx) ||
               unobserved_mask_levels_[child_height_idx].at(
                   {min_child_idx.x(), max_child_idx.y()}) ||
               unobserved_mask_levels_[child_height_idx].at(
                   {max_child_idx.x(), min_child_idx.y()}) ||
               unobserved_mask_levels_[child_height_idx].at(max_child_idx);
      }
    }
  } else {
    // Since the nodes at min_level_up are not direct neighbors we need to
    // go one level up and check all four parents there
    return unobserved_mask_levels_[parent_height_idx].at(min_parent_idx) ||
           unobserved_mask_levels_[parent_height_idx].at(
               {min_parent_idx.x(), max_parent_idx.y()}) ||
           unobserved_mask_levels_[parent_height_idx].at(
               {max_parent_idx.x(), min_parent_idx.y()}) ||
           unobserved_mask_levels_[parent_height_idx].at(max_parent_idx);
  }
}

inline UpdateType HierarchicalRangeBounds::getUpdateType(
    const Index2D& min_image_idx, const Index2D& max_image_idx,
    FloatingPoint range_min, FloatingPoint range_max) const {
  const Index2D min_image_idx_clamped = min_image_idx.cwiseMax(0);
  const Index2D max_image_idx_clamped =
      max_image_idx.cwiseMin(range_image_->getDimensions() - Index2D::Ones());

  const Bounds distance_bounds =
      getBounds(min_image_idx_clamped, max_image_idx_clamped);
  if (distance_bounds.upper < range_min) {
    return UpdateType::kFullyUnobserved;
  } else if (range_max < distance_bounds.lower) {
    if (min_image_idx != min_image_idx_clamped ||
        max_image_idx != max_image_idx_clamped ||
        hasUnobserved(min_image_idx_clamped, max_image_idx_clamped)) {
      return UpdateType::kFreeOrUnobserved;
    } else {
      return UpdateType::kFullyFree;
    }
  } else {
    return UpdateType::kPossiblyOccupied;
  }
}

inline Index2D HierarchicalRangeBounds::computeImageToPyramidScaleFactor(
    const ProjectorBase* projector) {
  if (projector) {
    const Vector2D stride = (projector->indexToImage(Index2D::Ones()) -
                             projector->indexToImage(Index2D::Zero()))
                                .cwiseAbs();
    return (stride / stride.minCoeff()).cwiseMin(2).cast<IndexElement>();
  } else {
    return Index2D::Ones();
  }
}

template <typename T, typename BinaryFunctor>
std::vector<Image<T>> HierarchicalRangeBounds::computeReducedPyramid(
    const Image<T>& range_image, BinaryFunctor reduction_functor, T init) {
  CHECK(!azimuth_wraps_pi_ || bit_ops::popcount(range_image.getNumColumns()))
      << "For LiDAR range images that wrap around horizontally (FoV of "
         "360deg), only column numbers that are exact powers of 2 are "
         "currently supported.";

  const Index2D range_image_dims = range_image.getDimensions();
  const Index2D range_image_dims_scaled =
      image_to_pyramid_scale_factor_.cwiseProduct(range_image_dims);
  const int max_num_halvings =
      int_math::log2_ceil(range_image_dims_scaled.maxCoeff());

  std::vector<Image<T>> pyramid;
  pyramid.reserve(max_num_halvings);
  for (int level_idx = 0; level_idx < max_num_halvings; ++level_idx) {
    // Initialize the current level
    const Index2D level_dims =
        int_math::div_exp2_ceil(range_image_dims_scaled, level_idx + 1);
    Image<T>& current_level =
        pyramid.emplace_back(level_dims.x(), level_dims.y(), init);

    // Reduce from the previous level or from the range image (for level 0)
    const Image<T>& previous_level =
        level_idx == 0 ? range_image : pyramid[level_idx - 1];
    const Index2D previous_level_dims = previous_level.getDimensions();

    for (const Index2D& idx :
         Grid<2>(Index2D::Zero(), level_dims - Index2D::Ones())) {
      Index2D min_child_idx = 2 * idx;
      Index2D max_child_idx = min_child_idx + Index2D::Ones();
      if (level_idx == 0) {
        min_child_idx =
            min_child_idx.cwiseQuotient(image_to_pyramid_scale_factor_);
        max_child_idx =
            max_child_idx.cwiseQuotient(image_to_pyramid_scale_factor_);
      }

      // Reduce the values in the 2x2 block of the previous level from
      // min_child_idx to max_child_idx while avoiding out-of-bounds access if
      // we're on the border. Where out-of-bounds access would occur, we
      // virtually pad the previous level with initial value 'init'.
      if ((min_child_idx.array() < previous_level_dims.array()).all()) {
        const T r00 = previous_level.at({min_child_idx.x(), min_child_idx.y()});
        if (max_child_idx.x() < previous_level_dims.x()) {
          const T r10 =
              previous_level.at({max_child_idx.x(), min_child_idx.y()});
          const T first_col_reduced = reduction_functor(r00, r10);
          if (max_child_idx.y() < previous_level_dims.y()) {
            const T r01 =
                previous_level.at({min_child_idx.x(), max_child_idx.y()});
            const T r11 =
                previous_level.at({max_child_idx.x(), max_child_idx.y()});
            const T second_col_reduced = reduction_functor(r01, r11);
            current_level.at(idx) =
                reduction_functor(first_col_reduced, second_col_reduced);
          } else if (azimuth_wraps_pi_ &&
                     max_child_idx.y() <= previous_level_dims.y()) {
            const T r01 = previous_level.at({min_child_idx.x(), 0});
            const T r11 = previous_level.at({max_child_idx.x(), 0});
            const T second_col_reduced = reduction_functor(r01, r11);
            current_level.at(idx) =
                reduction_functor(first_col_reduced, second_col_reduced);
          } else {
            current_level.at(idx) = reduction_functor(first_col_reduced, init);
          }
        } else if (max_child_idx.y() < previous_level_dims.y()) {
          const T r01 =
              previous_level.at({min_child_idx.x(), max_child_idx.y()});
          const T first_row_reduced = reduction_functor(r00, r01);
          current_level.at(idx) = reduction_functor(first_row_reduced, init);
        } else if (azimuth_wraps_pi_ &&
                   max_child_idx.y() <= previous_level_dims.y()) {
          const T r01 = previous_level.at({min_child_idx.x(), 0});
          const T first_row_reduced = reduction_functor(r00, r01);
          current_level.at(idx) = reduction_functor(first_row_reduced, init);
        } else {
          current_level.at(idx) = reduction_functor(r00, init);
        }
      } else {
        current_level.at(idx) = init;
      }
    }
  }
  return pyramid;
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HIERARCHICAL_RANGE_BOUNDS_INL_H_
