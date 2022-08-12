#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_BEAMWISE_INTEGRATOR_2D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_BEAMWISE_INTEGRATOR_2D_H_

#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/iterator/grid_iterator.h>

#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"

namespace wavemap {
class BeamwiseIntegrator2D : public PointcloudIntegrator2D {
 public:
  using PointcloudIntegrator2D::PointcloudIntegrator2D;

  void integratePointcloud(
      const PosedPointcloud<Point2D>& pointcloud) override {
    if (!isPointcloudValid(pointcloud)) {
      return;
    }

    const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
    MeasurementModelType measurement_model(min_cell_width);
    measurement_model.setStartPoint(pointcloud.getOrigin());

    for (const auto& end_point : pointcloud.getPointsGlobal()) {
      measurement_model.setEndPoint(end_point);
      if (!measurement_model.isMeasurementValid()) {
        continue;
      }

      FloatingPoint W_beam_heading_angle = 0.f;
      if (kEpsilon < measurement_model.getMeasuredDistance()) {
        const Point2D W_t_start_end_point =
            measurement_model.getEndPoint() - measurement_model.getStartPoint();
        W_beam_heading_angle =
            std::atan2(W_t_start_end_point.y(), W_t_start_end_point.x());
      }

      const Grid grid(measurement_model.getBottomLeftUpdateIndex(),
                      measurement_model.getTopRightUpdateIndex());
      for (const auto& index : grid) {
        const Point2D W_cell_center =
            convert::indexToCenterPoint(index, min_cell_width);
        const Point2D W_t_start_point_cell_center =
            W_cell_center - measurement_model.getStartPoint();

        // Compute the distance to the sensor
        const FloatingPoint d_C_cell = W_t_start_point_cell_center.norm();
        // Return early if the point is inside the sensor, beyond the beam's max
        // range, or far behind the surface
        if (d_C_cell < kEpsilon || MeasurementModelType::kRangeMax < d_C_cell ||
            measurement_model.getMeasuredDistance() +
                    MeasurementModelType::kRangeDeltaThresh <
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
        if (MeasurementModelType::kAngleThresh < cell_to_beam_angle) {
          continue;
        }

        // Compute the full measurement update
        const FloatingPoint update = MeasurementModelType::computeUpdate(
            d_C_cell, cell_to_beam_angle,
            measurement_model.getMeasuredDistance());
        if (kEpsilon < std::abs(update)) {
          occupancy_map_->addToCellValue(index, update);
        }
      }
    }
  }

 private:
  using MeasurementModelType = ContinuousVolumetricLogOdds<2>;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_BEAMWISE_INTEGRATOR_2D_H_
