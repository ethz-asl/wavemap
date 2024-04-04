#ifndef WAVEMAP_INTEGRATOR_RAY_TRACING_RAY_TRACING_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_RAY_TRACING_RAY_TRACING_INTEGRATOR_H_

#include <utility>

#include "wavemap/core/integrator/integrator_base.h"
#include "wavemap/core/integrator/measurement_model/constant_ray.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/iterate/ray_iterator.h"

namespace wavemap {
/**
 * Config struct for the ray-tracing integrator.
 */
struct RayTracingIntegratorConfig : ConfigBase<RayTracingIntegratorConfig, 2> {
  Meters<FloatingPoint> min_range = 0.5f;
  Meters<FloatingPoint> max_range = 20.f;

  static MemberMap memberMap;

  // Constructors
  RayTracingIntegratorConfig() = default;
  RayTracingIntegratorConfig(FloatingPoint min_range, FloatingPoint max_range)
      : min_range(min_range), max_range(max_range) {}

  bool isValid(bool verbose) const override;
};

class RayTracingIntegrator : public IntegratorBase {
 public:
  RayTracingIntegrator(const RayTracingIntegratorConfig& config,
                       MapBase::Ptr occupancy_map)
      : config_(config.checkValid()),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override;

 private:
  using MeasurementModelType = ConstantRay;

  const RayTracingIntegratorConfig config_;
  const MapBase::Ptr occupancy_map_;
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_RAY_TRACING_RAY_TRACING_INTEGRATOR_H_
