#ifndef WAVEMAP_2D_INTEGRATOR_POINT_INTEGRATOR_BEAM_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_POINT_INTEGRATOR_BEAM_INTEGRATOR_H_

#include <utility>

#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/iterator/grid_iterator.h"

namespace wavemap_2d {
class BeamIntegrator : public PointcloudIntegrator {
 public:
  explicit BeamIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override {
    if (!isPointcloudValid(pointcloud)) {
      return;
    }

    MeasurementModelType measurement_model(occupancy_map_->getResolution());
    measurement_model.setStartPoint(pointcloud.getOrigin());

    for (const auto& end_point : pointcloud.getPointsGlobal()) {
      measurement_model.setEndPoint(end_point);
      if (!measurement_model.isMeasurementValid()) {
        continue;
      }

      const Grid grid(measurement_model.getBottomLeftUpdateIndex(),
                      measurement_model.getTopRightUpdateIndex());
      for (const auto& index : grid) {
        const FloatingPoint update = measurement_model.computeUpdateAt(index);
        if (kEpsilon < std::abs(update)) {
          occupancy_map_->addToCellValue(index, update);
        }
      }
    }
  }

 private:
  using MeasurementModelType = BeamModel;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_POINT_INTEGRATOR_BEAM_INTEGRATOR_H_
