#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <limits>
#include <utility>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/datastructure_base.h"
#include "wavemap_2d/integrator/beam_model.h"
#include "wavemap_2d/integrator/fixed_logodds_model.h"
#include "wavemap_2d/datastructure/pointcloud.h"

namespace wavemap_2d {
class PointcloudIntegrator {
 public:
  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(DataStructureBase::Ptr occupancy_map,
                                MeasurementModelBase::Ptr measurement_model)
      : occupancy_map_(CHECK_NOTNULL(occupancy_map)),
        measurement_model_(CHECK_NOTNULL(measurement_model)) {}

  void integratePointcloud(const PosedPointcloud& pointcloud);

 protected:
  DataStructureBase::Ptr occupancy_map_;
  MeasurementModelBase::Ptr measurement_model_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
