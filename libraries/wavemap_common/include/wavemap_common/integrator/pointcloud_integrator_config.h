#ifndef WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_CONFIG_H_
#define WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_CONFIG_H_

namespace wavemap {
struct PointcloudIntegratorConfig {
  FloatingPoint min_range = 0.5f;
  FloatingPoint max_range = 20.f;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_CONFIG_H_
