#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_

#include <string>

#include "wavemap/common.h"
#include "wavemap/config/type_selector.h"

namespace wavemap {
struct UpdateType : TypeSelector<UpdateType> {
  using TypeSelector<UpdateType>::TypeSelector;

  enum Id : TypeId {
    kFullyUnobserved,
    kFullyFree,
    kFreeOrUnobserved,
    kPossiblyOccupied
  };

  static constexpr std::array names = {"fully_unobserved", "fully_free",
                                       "free_or_unobserved",
                                       "possibly_occupied"};
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_UPDATE_TYPE_H_
