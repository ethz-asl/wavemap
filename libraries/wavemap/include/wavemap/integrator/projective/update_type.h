#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_

#include <string>

#include "wavemap/common.h"
#include "wavemap/utils/type_utils.h"

namespace wavemap {
enum class UpdateType : int {
  kFullyUnobserved,
  kFullyFree,
  kFreeOrUnobserved,
  kPossiblyOccupied
};

inline std::string getUpdateTypeStr(UpdateType update_type) {
  static constexpr std::array kUpdateTypeStrs{"fully_unobserved", "fully_free",
                                              "free_or_unobserved",
                                              "possibly_occupied"};
  return kUpdateTypeStrs[to_underlying(update_type)];
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_
