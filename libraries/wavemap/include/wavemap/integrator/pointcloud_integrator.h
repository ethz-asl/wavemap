#ifndef WAVEMAP_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <memory>

#include "wavemap/common.h"
#include "wavemap/data_structure/pointcloud.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/utils/config_utils.h"

namespace wavemap {
struct PointcloudIntegratorConfig : ConfigBase<PointcloudIntegratorConfig> {
  FloatingPoint min_range = 0.5f;
  FloatingPoint max_range = 20.f;

  NdtreeIndexElement termination_height = 0;

  // Constructors
  PointcloudIntegratorConfig() = default;
  PointcloudIntegratorConfig(FloatingPoint min_range, FloatingPoint max_range)
      : min_range(min_range), max_range(max_range) {}

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

  static bool isPointcloudValid(const PosedPointcloud<Point<dim>>& pointcloud);
  static bool isMeasurementValid(const Point<dim>& C_end_point);

  static Point<dim> getEndPointOrMaxRange(const Point<dim>& W_start_point,
                                          const Point<dim>& W_end_point,
                                          FloatingPoint measured_distance,
                                          FloatingPoint max_range);
};
}  // namespace wavemap

#include "wavemap/integrator/impl/pointcloud_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
