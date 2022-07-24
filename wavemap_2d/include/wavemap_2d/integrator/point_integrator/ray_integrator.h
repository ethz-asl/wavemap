#ifndef WAVEMAP_2D_INTEGRATOR_POINT_INTEGRATOR_RAY_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_POINT_INTEGRATOR_RAY_INTEGRATOR_H_

#include <utility>

#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/iterator/ray_iterator.h"

namespace wavemap_2d {
class RayIntegrator : public PointcloudIntegrator {
 public:
  explicit RayIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override {
    if (!isPointcloudValid(pointcloud)) {
      return;
    }

    MeasurementModelType measurement_model(occupancy_map_->getMinCellWidth());
    measurement_model.setStartPoint(pointcloud.getOrigin());

    for (const auto& end_point : pointcloud.getPointsGlobal()) {
      measurement_model.setEndPoint(end_point);
      if (!measurement_model.isMeasurementValid()) {
        continue;
      }

      const Ray ray(measurement_model.getStartPoint(),
                    measurement_model.getEndPointOrMaxRange(),
                    occupancy_map_->getMinCellWidth());
      for (const auto& index : ray) {
        const FloatingPoint update = measurement_model.computeUpdateAt(index);
        occupancy_map_->addToCellValue(index, update);
      }
    }
  }

 private:
  using MeasurementModelType = FixedLogOddsModel;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_POINT_INTEGRATOR_RAY_INTEGRATOR_H_
