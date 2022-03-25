#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <limits>
#include <utility>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/generic/pointcloud.h"
#include "wavemap_2d/datastructure/volumetric/volumetric_datastructure.h"
#include "wavemap_2d/integrator/measurement_model/beam_model.h"
#include "wavemap_2d/integrator/measurement_model/fixed_logodds_model.h"

namespace wavemap_2d {
class PointcloudIntegrator {
 public:
  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(DataStructureBase::Ptr occupancy_map,
                                MeasurementModel::Ptr measurement_model)
      : occupancy_map_(CHECK_NOTNULL(occupancy_map)),
        measurement_model_(CHECK_NOTNULL(measurement_model)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud);

 protected:
  DataStructureBase::Ptr occupancy_map_;
  MeasurementModel::Ptr measurement_model_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
