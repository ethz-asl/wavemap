#ifndef WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <memory>

#include "wavemap_common/common.h"
#include "wavemap_common/data_structure/pointcloud.h"
#include "wavemap_common/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap_common/utils/config_utils.h"

namespace wavemap {
struct PointcloudIntegratorConfig : ConfigBase<PointcloudIntegratorConfig> {
  FloatingPoint min_range = 0.5f;
  FloatingPoint max_range = 20.f;

  bool isValid(bool verbose) const override;
  static PointcloudIntegratorConfig from(const param::Map& params);
};

template <int dim>
class PointcloudIntegrator {
 public:
  using Ptr = std::shared_ptr<PointcloudIntegrator>;

  PointcloudIntegrator() = delete;
  PointcloudIntegrator(
      const PointcloudIntegratorConfig& config,
      typename VolumetricDataStructureBase<dim>::Ptr occupancy_map)
      : config_(config.checkValid()),
        occupancy_map_(CHECK_NOTNULL(occupancy_map)) {}
  virtual ~PointcloudIntegrator() = default;

  virtual void integratePointcloud(
      const PosedPointcloud<Point<dim>>& pointcloud) = 0;

 protected:
  const PointcloudIntegratorConfig config_;

  typename VolumetricDataStructureBase<dim>::Ptr occupancy_map_;

  static bool isPointcloudValid(const PosedPointcloud<Point<dim>>& pointcloud) {
    if (const Point<dim>& origin = pointcloud.getOrigin(); origin.hasNaN()) {
      LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                      "contains NaNs:\n"
                   << origin;
      return false;
    }

    return true;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
