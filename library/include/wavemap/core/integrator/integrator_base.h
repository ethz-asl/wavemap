#ifndef WAVEMAP_CORE_INTEGRATOR_INTEGRATOR_BASE_H_
#define WAVEMAP_CORE_INTEGRATOR_INTEGRATOR_BASE_H_

#include <memory>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/config/config_base.h"
#include "wavemap/core/config/type_selector.h"
#include "wavemap/core/data_structure/image.h"
#include "wavemap/core/data_structure/pointcloud.h"

namespace wavemap {
struct IntegratorType : TypeSelector<IntegratorType> {
  using TypeSelector<IntegratorType>::TypeSelector;

  enum Id : TypeId {
    kRayTracingIntegrator,
    kFixedResolutionIntegrator,
    kCoarseToFineIntegrator,
    kWaveletIntegrator,
    kHashedWaveletIntegrator,
    kHashedChunkedWaveletIntegrator
  };

  static constexpr std::array names = {
      "ray_tracing_integrator",    "fixed_resolution_integrator",
      "coarse_to_fine_integrator", "wavelet_integrator",
      "hashed_wavelet_integrator", "hashed_chunked_wavelet_integrator"};
};

class IntegratorBase {
 public:
  using Ptr = std::shared_ptr<IntegratorBase>;

  IntegratorBase() = default;
  virtual ~IntegratorBase() = default;

  virtual void integrate(const PosedPointcloud<>& pointcloud) = 0;
  virtual void integrate(const PosedImage<>& range_image) = 0;

 protected:
  static bool isPointcloudValid(const PosedPointcloud<>& pointcloud);
  static bool isMeasurementValid(const Point3D& C_end_point);

  static Point3D getEndPointOrMaxRange(const Point3D& W_start_point,
                                       const Point3D& W_end_point,
                                       FloatingPoint measured_distance,
                                       FloatingPoint max_range);
};
}  // namespace wavemap

#include "wavemap/core/integrator/impl/integrator_base_inl.h"

#endif  // WAVEMAP_CORE_INTEGRATOR_INTEGRATOR_BASE_H_
