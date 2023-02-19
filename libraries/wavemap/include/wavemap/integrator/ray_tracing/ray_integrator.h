#ifndef WAVEMAP_INTEGRATOR_RAY_TRACING_RAY_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_RAY_TRACING_RAY_INTEGRATOR_H_

#include <utility>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/measurement_model/range_only/constant_1d_log_odds.h"
#include "wavemap/integrator/pointcloud_integrator.h"
#include "wavemap/iterator/ray_iterator.h"

namespace wavemap {
class RayIntegrator : public PointcloudIntegrator {
 public:
  using PointcloudIntegrator::PointcloudIntegrator;

  void integratePointcloud(
      const PosedPointcloud<Point3D>& pointcloud) override {
    if (!Base::isPointcloudValid(pointcloud)) {
      return;
    }

    const FloatingPoint min_cell_width =
        Base::occupancy_map_->getMinCellWidth();
    const Point3D W_start_point = pointcloud.getOrigin();

    MeasurementModelType measurement_model(min_cell_width);
    measurement_model.setStartPoint(W_start_point);

    for (const auto& W_end_point : pointcloud.getPointsGlobal()) {
      measurement_model.setEndPoint(W_end_point);

      if (!Base::isMeasurementValid(W_end_point - W_start_point)) {
        continue;
      }

      const FloatingPoint measured_distance =
          (W_start_point - W_end_point).norm();
      const Point3D W_end_point_truncated = Base::getEndPointOrMaxRange(
          W_start_point, W_end_point, measured_distance,
          Base::config_.max_range);
      const Ray ray(W_start_point, W_end_point_truncated, measured_distance);
      for (const auto& index : ray) {
        const FloatingPoint update = measurement_model.computeUpdate(index);
        Base::occupancy_map_->addToCellValue(index, update);
      }
    }
  }

 private:
  using Base = PointcloudIntegrator;
  using MeasurementModelType = Constant1DLogOdds;
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_RAY_TRACING_RAY_INTEGRATOR_H_
