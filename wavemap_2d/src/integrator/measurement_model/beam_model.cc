#include "wavemap_2d/integrator/measurement_model/beam_model.h"

#include "wavemap_2d/indexing/index_conversions.h"

namespace wavemap_2d {
Index BeamModel::getBottomLeftUpdateIndex() const {
  const Point bottom_left_point = W_start_point_.cwiseMin(
      getEndPointOrMaxRange() - Point::Constant(max_lateral_component_));
  return computeFloorIndexForPoint(bottom_left_point, resolution_inv_);
}

Index BeamModel::getTopRightUpdateIndex() const {
  const Point top_right_point = W_start_point_.cwiseMax(
      getEndPointOrMaxRange() + Point::Constant(max_lateral_component_));
  return computeCeilIndexForPoint(top_right_point, resolution_inv_);
}
}  // namespace wavemap_2d
