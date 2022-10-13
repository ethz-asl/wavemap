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

    MeasurementModelType measurement_model(
        Base::occupancy_map_->getMinCellWidth());
    measurement_model.setStartPoint(pointcloud.getOrigin());

    for (const auto& end_point : pointcloud.getPointsGlobal()) {
      measurement_model.setEndPoint(end_point);
      if (!measurement_model.isMeasurementValid()) {
        continue;
      }

      const Ray ray(
          measurement_model.getStartPoint(),
          measurement_model.getEndPointOrMaxRange(Base::config_.max_range),
          Base::occupancy_map_->getMinCellWidth());
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
