#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_BEAMWISE_INTEGRATOR_2D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_BEAMWISE_INTEGRATOR_2D_H_

#include <utility>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"

namespace wavemap {
class BeamwiseIntegrator2D : public PointcloudIntegrator2D {
 public:
  BeamwiseIntegrator2D(const PointcloudIntegratorConfig& config,
                       ContinuousVolumetricLogOdds<2> measurement_model,
                       VolumetricDataStructure2D::Ptr occupancy_map)
      : PointcloudIntegrator2D(config, std::move(occupancy_map)),
        measurement_model_(std::move(measurement_model)) {}

  void integratePointcloud(const PosedPointcloud<Point2D>& pointcloud) override;

 private:
  const ContinuousVolumetricLogOdds<2> measurement_model_;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_BEAMWISE_INTEGRATOR_2D_H_
