#include "wavemap_2d/integrator/measurement_model/beam_model.h"

namespace wavemap_2d {
Index BeamModel::getBottomLeftUpdateIndex() const {
  const Point bottom_left_point = W_start_point_.cwiseMin(
      W_end_point_ - Point::Constant(max_lateral_component_));
  return computeFloorIndexForPoint(bottom_left_point, resolution_inv_);
}

Index BeamModel::getTopRightUpdateIndex() const {
  const Point top_right_point = W_start_point_.cwiseMax(
      W_end_point_ + Point::Constant(max_lateral_component_));
  return computeCeilIndexForPoint(top_right_point, resolution_inv_);
}

FloatingPoint BeamModel::computeUpdateAt(const Index& index) const {
  const Point W_cell_center = computeCenterFromIndex(index, resolution_);
  const Point C_cell_center = W_cell_center - W_start_point_;

  const FloatingPoint distance = C_cell_center.norm();
  if (kRangeMax < distance) {
    return 0.f;
  }

  if (kEpsilon < distance &&
      distance <= measured_distance_ + kRangeDeltaThresh) {
    const FloatingPoint dot_prod_normalized =
        C_cell_center.dot(C_end_point_normalized_) / distance;
    const FloatingPoint angle = std::acos(dot_prod_normalized);
    if (angle <= kAngleThresh) {
      const FloatingPoint f = (distance - measured_distance_) / kRangeSigma;
      const FloatingPoint g = angle / kAngleSigma;
      const FloatingPoint range_contrib = Qcdf(f) - 0.5f * Qcdf(f - 3.f) - 0.5f;
      const FloatingPoint angle_contrib = Qcdf(g + 3.f) - Qcdf(g - 3.f);
      const FloatingPoint contribs = range_contrib * angle_contrib;
      const FloatingPoint scaled_contribs =
          ((contribs < 0.f) ? kFreeScaling * contribs : kOccScaling * contribs);
      const FloatingPoint p = scaled_contribs + 0.5f;
      const FloatingPoint log_odds = std::log(p / (1.f - p));
      return log_odds;
    }
  }
  return 0.f;
}

void BeamModel::updateMap(DataStructureBase& map) const {
  if (!kUseClearing && exceedsMaxRange()) {
    return;
  }

  const Grid grid(getUpdateMinIndex(), getUpdateMaxIndex());
  for (const auto& index : grid) {
    const FloatingPoint update = computeUpdateAt(index);
    if (kEpsilon < std::abs(update)) {
      map.addToCellValue(index, update);
    }
  }
}

FloatingPoint BeamModel::Qcdf(FloatingPoint t) {
  if (t < -3.f) {
    return 0.f;
  } else if (-3.f <= t && t <= -1.f) {
    return 1.f / 48.f * static_cast<FloatingPoint>(std::pow(3.f + t, 3));
  } else if (-1.f < t && t < 1.f) {
    return 1.f / 2.f + 1.f / 24.f * t * (3.f + t) * (3.f - t);
  } else if (1.f <= t && t <= 3.f) {
    return 1.f - 1.f / 48.f * static_cast<FloatingPoint>(std::pow(3.f - t, 3));
  } else {
    return 1.f;
  }
}

Index BeamModel::getUpdateMinIndex() const {
  const Point bottom_left_point = W_start_point_.cwiseMin(
      getEndPointOrMaxRange() - Point::Constant(max_lateral_component_));
  return computeFloorIndexForPoint(bottom_left_point, resolution_inv_);
}

Index BeamModel::getUpdateMaxIndex() const {
  const Point top_right_point = W_start_point_.cwiseMax(
      getEndPointOrMaxRange() + Point::Constant(max_lateral_component_));
  return computeCeilIndexForPoint(top_right_point, resolution_inv_);
}
}  // namespace wavemap_2d
