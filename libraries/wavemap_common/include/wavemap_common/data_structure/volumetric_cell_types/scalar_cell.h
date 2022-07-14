#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_SCALAR_CELL_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_SCALAR_CELL_H_

#include <algorithm>
#include <limits>

#include "wavemap_common/common.h"
#include "wavemap_common/data_structure/volumetric_cell_types/cell_traits.h"

namespace wavemap {
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

  static Specialized threshold(Specialized cell_value) { return cell_value; }
  static Specialized add(Specialized cell_value, Specialized update) {
    return threshold(cell_value + update);
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

  static Specialized threshold(Specialized cell_value) {
    return std::max(kLowerBound, cell_value);
  }
  static Specialized add(Specialized cell_value, Specialized update) {
    return threshold(cell_value + update);
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

  static Specialized threshold(Specialized cell_value) {
    return std::min(cell_value, kUpperBound);
  }
  static Specialized add(Specialized cell_value, Specialized update) {
    return threshold(cell_value + update);
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

  static Specialized threshold(Specialized cell_value) {
    return std::clamp(cell_value, kLowerBound, kUpperBound);
  }
  static Specialized add(Specialized cell_value, Specialized update) {
    return threshold(cell_value + update);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_SCALAR_CELL_H_
