#ifndef WAVEMAP_INTEGRATOR_INTEGRATOR_BASE_H_
#define WAVEMAP_INTEGRATOR_INTEGRATOR_BASE_H_

#include <memory>
#include <utility>

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/config/type_selector.h"
#include "wavemap/data_structure/pointcloud.h"

namespace wavemap {
struct IntegratorType : TypeSelector<IntegratorType> {
  using TypeSelector<IntegratorType>::TypeSelector;

  enum Id : TypeId {
    kRayTracingIntegrator,
    kFixedResolutionIntegrator,
    kCoarseToFineIntegrator,
    kWaveletIntegrator
  };

  static constexpr std::array names = {
      "ray_tracing_integrator", "fixed_resolution_integrator",
      "coarse_to_fine_integrator", "wavelet_integrator"};
};

class IntegratorBase {
 public:
  using Ptr = std::shared_ptr<IntegratorBase>;

  IntegratorBase() = default;
  virtual ~IntegratorBase() = default;

  virtual void integratePointcloud(
      const PosedPointcloud<Point3D>& pointcloud) = 0;

 protected:
  static bool isPointcloudValid(const PosedPointcloud<Point3D>& pointcloud);
  static bool isMeasurementValid(const Point3D& C_end_point);

  static Point3D getEndPointOrMaxRange(const Point3D& W_start_point,
                                       const Point3D& W_end_point,
                                       FloatingPoint measured_distance,
                                       FloatingPoint max_range);
};
}  // namespace wavemap

#include "wavemap/integrator/impl/integrator_base_inl.h"

#endif  // WAVEMAP_INTEGRATOR_INTEGRATOR_BASE_H_
