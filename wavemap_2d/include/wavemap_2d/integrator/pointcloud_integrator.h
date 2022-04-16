#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <limits>
#include <utility>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/pointcloud.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/integrator/measurement_model/beam_model.h"
#include "wavemap_2d/integrator/measurement_model/fixed_logodds_model.h"

namespace wavemap_2d {
class PointcloudIntegrator {
 public:
  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : occupancy_map_(CHECK_NOTNULL(occupancy_map)) {}
  virtual ~PointcloudIntegrator() = default;

  virtual void integratePointcloud(const PosedPointcloud<>& pointcloud) = 0;

 protected:
  VolumetricDataStructure::Ptr occupancy_map_;

  static bool isPointcloudValid(const PosedPointcloud<>& pointcloud);
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
