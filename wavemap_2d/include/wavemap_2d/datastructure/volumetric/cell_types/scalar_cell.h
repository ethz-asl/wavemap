#ifndef WAVEMAP_2D_DATASTRUCTURE_VOLUMETRIC_CELL_TYPES_SCALAR_CELL_H_
#define WAVEMAP_2D_DATASTRUCTURE_VOLUMETRIC_CELL_TYPES_SCALAR_CELL_H_

#include <algorithm>
#include <limits>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/volumetric/cell_types/cell_traits.h"

namespace wavemap_2d {
using BoundType = int;
using ScalarCell = CellTraits<FloatingPoint, FloatingPoint, uint16_t>;

struct UnboundedScalarCell : ScalarCell {
  static constexpr bool hasLowerBound = false;
  static constexpr Specialized kLowerBound =
      std::numeric_limits<Specialized>::lowest();
  static constexpr bool hasUpperBound = false;
  static constexpr Specialized kUpperBound =
      std::numeric_limits<Specialized>::max();

  static constexpr bool isFullyBounded = false;
  static constexpr Specialized kSpecializedToBaseIntScalingFactor = 1.f;

  static Specialized add(Specialized cell_value, Specialized update) {
    return cell_value + update;
  }
};

template <BoundType lower_bound>
struct LowerBoundedScalarCell : ScalarCell {
  static constexpr bool hasLowerBound = true;
  static constexpr Specialized kLowerBound =
      static_cast<Specialized>(lower_bound);
  static constexpr bool hasUpperBound = false;
  static constexpr Specialized kUpperBound =
      std::numeric_limits<Specialized>::max();

  static constexpr bool isFullyBounded = false;
  static constexpr Specialized kSpecializedToBaseIntScalingFactor = 1.f;

  static Specialized add(Specialized cell_value, Specialized update) {
    return std::max(kLowerBound, cell_value + update);
  }
};

template <BoundType upper_bound>
struct UpperBoundedScalarCell : ScalarCell {
  static constexpr bool hasLowerBound = false;
  static constexpr Specialized kLowerBound =
      std::numeric_limits<Specialized>::lowest();
  static constexpr bool hasUpperBound = true;
  static constexpr Specialized kUpperBound =
      static_cast<Specialized>(upper_bound);

  static constexpr bool isFullyBounded = false;
  static constexpr Specialized kSpecializedToBaseIntScalingFactor = 1.f;

  static Specialized add(Specialized cell_value, Specialized update) {
    return std::min(cell_value + update, kUpperBound);
  }
};

template <BoundType lower_bound, BoundType upper_bound>
struct BoundedScalarCell : ScalarCell {
  static constexpr bool hasLowerBound = true;
  static constexpr Specialized kLowerBound =
      static_cast<Specialized>(lower_bound);
  static constexpr bool hasUpperBound = true;
  static constexpr Specialized kUpperBound =
      static_cast<Specialized>(upper_bound);

  static constexpr bool isFullyBounded = true;
  static constexpr Specialized kSpecializedToBaseIntScalingFactor =
      (static_cast<Specialized>(std::numeric_limits<BaseInt>::max()) -
       static_cast<Specialized>(std::numeric_limits<BaseInt>::lowest())) /
      (kUpperBound - kLowerBound);

  static Specialized add(Specialized cell_value, Specialized update) {
    return std::clamp(cell_value + update, kLowerBound, kUpperBound);
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_VOLUMETRIC_CELL_TYPES_SCALAR_CELL_H_
