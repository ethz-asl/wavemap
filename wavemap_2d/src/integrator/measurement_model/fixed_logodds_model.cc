#include "wavemap_2d/integrator/measurement_model/fixed_logodds_model.h"

namespace wavemap_2d {
Index FixedLogOddsModel::getBottomLeftUpdateIndex() const {
  const Point bottom_left_point = W_start_point_.cwiseMin(W_end_point_);
  return computeFloorIndexForPoint(bottom_left_point, resolution_inv_);
}

Index FixedLogOddsModel::getTopRightUpdateIndex() const {
  const Point top_right_point = W_start_point_.cwiseMax(W_end_point_);
  return computeCeilIndexForPoint(top_right_point, resolution_inv_);
}

void FixedLogOddsModel::updateMap(DataStructureBase& map) const {
  if (!kUseClearing && exceedsMaxRange()) {
    return;
  }

  const Ray ray(W_start_point_, getEndPointOrMaxRange(), map.getResolution());
  for (const auto& index : ray) {
    const FloatingPoint update = computeUpdateAt(index);
    map.addToCellValue(index, update);
  }
}
}  // namespace wavemap_2d
