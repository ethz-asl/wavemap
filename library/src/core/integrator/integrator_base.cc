#include "wavemap/core/integrator/integrator_base.h"

namespace wavemap {
bool IntegratorBase::isPoseValid(const Transformation3D& T_W_C) {
  if (T_W_C.getPosition().hasNaN()) {
    LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                    "contains NaNs:\n"
                 << T_W_C.getPosition();
    return false;
  }

  return true;
}
}  // namespace wavemap
