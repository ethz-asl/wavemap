#ifndef WAVEMAP_2D_DATASTRUCTURE_CELL_H_
#define WAVEMAP_2D_DATASTRUCTURE_CELL_H_

#include <limits>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
using BoundType = int;
template <typename SpecializedType, typename BaseFloatType,
          typename BaseIntType,
          BoundType lower_bound = std::numeric_limits<BoundType>::lowest(),
          BoundType upper_bound = std::numeric_limits<BoundType>::max()>
struct CellTraits {
  using Specialized = SpecializedType;
  using BaseFloat = BaseFloatType;
  using BaseInt = BaseIntType;

  static constexpr SpecializedType kLowerBound =
      static_cast<SpecializedType>(lower_bound);
  static constexpr SpecializedType kUpperBound =
      static_cast<SpecializedType>(upper_bound);
  static constexpr bool hasLowerBound =
      (kLowerBound != std::numeric_limits<BoundType>::lowest());
  static constexpr bool hasUpperBound =
      (kUpperBound != std::numeric_limits<BoundType>::max());
  static constexpr bool isFullyBounded = (hasLowerBound && hasUpperBound);

  static SpecializedType add(SpecializedType cell_value,
                             SpecializedType update) {
    if (hasLowerBound && hasUpperBound) {
      return std::clamp(cell_value + update, kLowerBound, kUpperBound);
    } else if (hasUpperBound) {
      return std::min(cell_value + update, kUpperBound);
    } else if (hasLowerBound) {
      return std::max(kLowerBound, cell_value + update);
    } else {
      return cell_value + update;
    }
  }

  static constexpr SpecializedType kSpecializedToBaseIntScalingFactor =
      (isFullyBounded)
          ? (static_cast<SpecializedType>(std::numeric_limits<BaseInt>::max()) -
             static_cast<SpecializedType>(
                 std::numeric_limits<BaseInt>::lowest())) /
                (kUpperBound - kLowerBound)
          : 1.f;
};

struct UnboundedCell : CellTraits<FloatingPoint, FloatingPoint, int16_t> {};

template <BoundType lower_bound = -2, BoundType upper_bound = 4>
struct SaturatingCell : CellTraits<FloatingPoint, FloatingPoint, uint16_t,
                                   lower_bound, upper_bound> {};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_CELL_H_
