#include "wavemap_2d/integrator/fixed_logodds_model.h"

namespace wavemap_2d {
Index FixedLogOddsModel::getBottomLeftUpdateIndex() {
  const Point bottom_left_point = W_start_point_.cwiseMin(W_end_point_);
  const Point bottom_left_point_scaled = bottom_left_point * resolution_inv_;
  return bottom_left_point_scaled.array().floor().cast<IndexElement>();
}

Index FixedLogOddsModel::getTopRightUpdateIndex() {
  const Point top_right_point = W_start_point_.cwiseMax(W_end_point_);
  const Point top_right_point_scaled = top_right_point * resolution_inv_;
  return top_right_point_scaled.array().ceil().cast<IndexElement>();
}

void FixedLogOddsModel::updateMap(DataStructureBase& map) {
  if (exceedsMaxRange()) {
    return;
  }

  const Ray ray(W_start_point_, W_end_point_, map.getResolution());
  for (const auto& index : ray) {
    const FloatingPoint update = computeUpdateAt(index);
    map.addToCellValue(index, update);
  }
}
}  // namespace wavemap_2d
