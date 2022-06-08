#include "wavemap_2d/integrator/measurement_model/beam_model.h"

#include "wavemap_2d/indexing/index_conversions.h"

namespace wavemap_2d {
Index BeamModel::getBottomLeftUpdateIndex() const {
  const Point bottom_left_point = W_start_point_.cwiseMin(
      getEndPointOrMaxRange() - Point::Constant(max_lateral_component_));
  return convert::pointToFloorIndex(bottom_left_point, min_cell_width_inv_);
}

Index BeamModel::getTopRightUpdateIndex() const {
  const Point top_right_point = W_start_point_.cwiseMax(
      getEndPointOrMaxRange() + Point::Constant(max_lateral_component_));
  return convert::pointToCeilIndex(top_right_point, min_cell_width_inv_);
}
}  // namespace wavemap_2d
