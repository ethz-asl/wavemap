#ifndef WAVEMAP_UTILS_QUERY_OCCUPANCY_H_
#define WAVEMAP_UTILS_QUERY_OCCUPANCY_H_

#include "wavemap/common.h"
#include "wavemap/config/type_selector.h"

namespace wavemap {
struct Occupancy : TypeSelector<Occupancy> {
  using TypeSelector<Occupancy>::TypeSelector;
  using Mask = uint8_t;

  enum Id : TypeId { kFree, kOccupied, kUnobserved, kObserved, kAny };
  static constexpr std::array names = {"free", "occupied", "unobserved",
                                       "observed", "any"};

  // NOTE: For usage examples, please refer to the OccupancyClassifier class.
  static constexpr Mask toMask(Id type_id);
  static constexpr Mask toMask(bool free, bool occupied, bool unobserved);
  constexpr Mask toMask() const { return toMask(static_cast<Id>(id_)); }
};
}  // namespace wavemap

#include "wavemap/utils/query/impl/occupancy_inl.h"

#endif  // WAVEMAP_UTILS_QUERY_OCCUPANCY_H_
