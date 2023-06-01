#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_

#include <string>

#include "wavemap_common/common.h"
#include "wavemap_common/utils/type_utils.h"

namespace wavemap {
enum class UpdateType : int {
  kFullyUnobserved,
  kFreeOrUnobserved,
  kPossiblyOccupied
};

inline std::string getUpdateTypeStr(UpdateType update_type) {
  static constexpr std::array kUpdateTypeStrs{
      "fully_unobserved", "free_or_unobserved", "possibly_occupied"};
  return kUpdateTypeStrs[to_underlying(update_type)];
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_
