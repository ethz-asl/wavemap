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
      const PosedPointcloud<Point2D, Transformation2D>& pointcloud) override {
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

      const Grid grid(measurement_model.getBottomLeftUpdateIndex(),
                      measurement_model.getTopRightUpdateIndex());
      for (const auto& index : grid) {
        const FloatingPoint update = measurement_model.computeUpdate(index);
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
