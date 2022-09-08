#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTIVE_INTERSECTION_TYPE_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTIVE_INTERSECTION_TYPE_H_

#include <string>

#include "wavemap_common/common.h"
#include "wavemap_common/utils/type_utils.h"

namespace wavemap {
enum class IntersectionType : int {
  kFullyUnknown,
  kFreeOrUnknown,
  kPossiblyOccupied
};

inline std::string getIntersectionTypeStr(IntersectionType intersection_type) {
  static constexpr std::array<const char*, 3> kIntersectionTypeStrs(
      {"fully_unknown", "free_or_unknown", "possibly_occupied"});
  return kIntersectionTypeStrs[to_underlying(intersection_type)];
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTIVE_INTERSECTION_TYPE_H_
