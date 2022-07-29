#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_

#include <string>

#include <wavemap_common/utils/type_utils.h>

#include "wavemap_2d/integrator/pointcloud_integrator.h"

namespace wavemap {
enum class PointcloudIntegratorType : int {
  kSingleRayIntegrator,
  kSingleBeamIntegrator,
  kFixedResolutionScanIntegrator,
  kCoarseToFineScanIntegrator,
  kWaveletScanIntegrator
};
constexpr std::array<const char*, 5> kPointcloudIntegratorTypeStrs = {
    "single_ray_integrator", "single_beam_integrator",
    "fixed_resolution_scan_integrator", "coarse_to_fine_scan_integrator",
    "wavelet_scan_integrator"};
std::string getPointcloudIntegratorTypeStr(
    PointcloudIntegratorType intersection_type) {
  return kPointcloudIntegratorTypeStrs[to_underlying(intersection_type)];
}

class PointcloudIntegratorFactory {
 public:
  static PointcloudIntegrator::Ptr create(
      const std::string& integrator_type_name,
      VolumetricDataStructure2D::Ptr occupancy_map,
      std::optional<PointcloudIntegratorType> default_integrator_type =
          std::nullopt);

  static PointcloudIntegrator::Ptr create(
      PointcloudIntegratorType integrator_type,
      VolumetricDataStructure2D::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_
