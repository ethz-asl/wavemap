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
  static bool isMeasurementValid(const Point<dim>& W_end_point,
                                 FloatingPoint measured_distance) {
    if (W_end_point.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << W_end_point;
      return false;
    }
    if (measured_distance < kEpsilon) {
      return false;
    }
    if (1e3 < measured_distance) {
      LOG(INFO) << "Skipping measurement with suspicious length: "
                << measured_distance;
      return false;
    }
    return true;
  }
  static Point<dim> getEndPointOrMaxRange(const Point<dim>& W_start_point,
                                          const Point<dim>& W_end_point,
                                          FloatingPoint measured_distance,
                                          FloatingPoint max_range) {
    if (max_range < measured_distance) {
      return W_start_point +
             max_range / measured_distance * (W_end_point - W_start_point);
    } else {
      return W_end_point;
    }
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
