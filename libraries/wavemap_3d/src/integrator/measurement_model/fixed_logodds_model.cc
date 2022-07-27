#include "wavemap_3d/integrator/measurement_model/fixed_logodds_model.h"

#include <wavemap_common/indexing/index_conversions.h>

namespace wavemap {
Index3D FixedLogOddsModel::getBottomLeftUpdateIndex() const {
  const Point3D bottom_left_point = W_start_point_.cwiseMin(W_end_point_);
  return convert::pointToFloorIndex(bottom_left_point, min_cell_width_inv_);
}

Index3D FixedLogOddsModel::getTopRightUpdateIndex() const {
  const Point3D top_right_point = W_start_point_.cwiseMax(W_end_point_);
  return convert::pointToCeilIndex(top_right_point, min_cell_width_inv_);
}
}  // namespace wavemap
