#include "wavemap_2d/integrator/measurement_model/beam_model.h"

#include <wavemap_common/indexing/index_conversions.h>

namespace wavemap {
Index2D BeamModel::getBottomLeftUpdateIndex() const {
  const Point2D bottom_left_point = W_start_point_.cwiseMin(
      getEndPointOrMaxRange() - Point2D::Constant(max_lateral_component_));
  return convert::pointToFloorIndex(bottom_left_point, min_cell_width_inv_);
}

Index2D BeamModel::getTopRightUpdateIndex() const {
  const Point2D top_right_point = W_start_point_.cwiseMax(
      getEndPointOrMaxRange() + Point2D::Constant(max_lateral_component_));
  return convert::pointToCeilIndex(top_right_point, min_cell_width_inv_);
}
}  // namespace wavemap
