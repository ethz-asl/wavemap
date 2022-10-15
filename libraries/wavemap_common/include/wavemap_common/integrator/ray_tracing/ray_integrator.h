#ifndef WAVEMAP_COMMON_INTEGRATOR_RAY_TRACING_RAY_INTEGRATOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_RAY_TRACING_RAY_INTEGRATOR_H_

#include <utility>

#include "wavemap_common/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap_common/integrator/measurement_model/range_only/constant_1d_log_odds.h"
#include "wavemap_common/integrator/pointcloud_integrator.h"
#include "wavemap_common/iterator/ray_iterator.h"

namespace wavemap {
template <int dim>
class RayIntegrator : public PointcloudIntegrator<dim> {
 public:
  using PointcloudIntegrator<dim>::PointcloudIntegrator;

  void integratePointcloud(
      const PosedPointcloud<Point<dim>>& pointcloud) override {
    if (!Base::isPointcloudValid(pointcloud)) {
      return;
    }

    const FloatingPoint min_cell_width =
        Base::occupancy_map_->getMinCellWidth();
    const Point<dim> W_start_point = pointcloud.getOrigin();

    MeasurementModelType measurement_model(min_cell_width);
    measurement_model.setStartPoint(W_start_point);

    for (const auto& W_end_point : pointcloud.getPointsGlobal()) {
      measurement_model.setEndPoint(W_end_point);

      const FloatingPoint measured_distance =
          (W_start_point - W_end_point).norm();
      if (!Base::isMeasurementValid(W_end_point, measured_distance)) {
        continue;
      }

      const Point<dim> W_end_point_truncated = Base::getEndPointOrMaxRange(
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
  using Base = PointcloudIntegrator<dim>;
  using MeasurementModelType = Constant1DLogOdds<dim>;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_RAY_TRACING_RAY_INTEGRATOR_H_
