#include "wavemap_2d/integrator/projective/beamwise_integrator_2d.h"

#include <algorithm>

#include <wavemap_common/iterator/grid_iterator.h>

namespace wavemap {
void BeamwiseIntegrator2D::integratePointcloud(
    const PosedPointcloud<Point2D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;

  const Point2D& W_start_point = pointcloud.getOrigin();
  for (const auto& W_end_point : pointcloud.getPointsGlobal()) {
    const FloatingPoint measured_distance =
        (W_end_point - W_start_point).norm();
    if (!isMeasurementValid(W_end_point - W_start_point)) {
      continue;
    }

    FloatingPoint W_beam_heading_angle = 0.f;
    if (kEpsilon < measured_distance) {
      const Point2D W_t_start_end_point = W_end_point - W_start_point;
      W_beam_heading_angle =
          std::atan2(W_t_start_end_point.y(), W_t_start_end_point.x());
    }

    const Point2D W_end_point_truncated = getEndPointOrMaxRange(
        W_start_point, W_end_point, measured_distance, config_.max_range);
    const FloatingPoint measured_distance_truncated =
        std::min(measured_distance, config_.max_range);
    const Index2D bottom_left_update_index =
        measurement_model_.getBottomLeftUpdateIndex(
            W_start_point, W_end_point_truncated, measured_distance_truncated,
            min_cell_width_inv);
    const Index2D top_right_update_index =
        measurement_model_.getTopRightUpdateIndex(
            W_start_point, W_end_point_truncated, measured_distance_truncated,
            min_cell_width_inv);

    const Grid grid(bottom_left_update_index, top_right_update_index);
    for (const auto& index : grid) {
      const Point2D W_cell_center =
          convert::indexToCenterPoint(index, min_cell_width);
      const Point2D W_t_start_point_cell_center = W_cell_center - W_start_point;

      // Compute the distance to the sensor
      const FloatingPoint d_C_cell = W_t_start_point_cell_center.norm();
      // Return early if the point is inside the sensor, beyond the beam's max
      // range, or far behind the surface
      if (d_C_cell < config_.min_range || config_.max_range < d_C_cell ||
          measured_distance +
                  measurement_model_.getRangeThresholdBehindSurface() <
              d_C_cell) {
        continue;
      }

      // Compute the angle w.r.t. the ray
      const FloatingPoint W_cell_heading_angle = std::atan2(
          W_t_start_point_cell_center.y(), W_t_start_point_cell_center.x());
      const FloatingPoint cell_to_beam_angle =
          std::abs(W_cell_heading_angle - W_beam_heading_angle);

      // Return early if the point is outside the beam's non-zero angular
      // region
      if (measurement_model_.getAngleThreshold() < cell_to_beam_angle) {
        continue;
      }

      // Compute the full measurement update
      const FloatingPoint update = measurement_model_.computeUpdate(
          d_C_cell, cell_to_beam_angle, measured_distance);
      if (kEpsilon < std::abs(update)) {
        occupancy_map_->addToCellValue(index, update);
      }
    }
  }
}
}  // namespace wavemap
