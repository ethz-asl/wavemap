#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_2D_FACTORY_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_2D_FACTORY_H_

#include <string>

#include <wavemap_common/utils/type_utils.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"

namespace wavemap {
enum class PointcloudIntegrator2DType : int {
  kSingleRayIntegrator,
  kSingleBeamIntegrator,
  kFixedResolutionScanIntegrator,
  kCoarseToFineScanIntegrator,
  kWaveletScanIntegrator
};
constexpr std::array kPointcloudIntegrator2DTypeStrs = {
    "single_ray_integrator", "single_beam_integrator",
    "fixed_resolution_scan_integrator", "coarse_to_fine_scan_integrator",
    "wavelet_scan_integrator"};
std::string getPointcloudIntegrator2DTypeStr(
    PointcloudIntegrator2DType intersection_type) {
  return kPointcloudIntegrator2DTypeStrs[to_underlying(intersection_type)];
}

class PointcloudIntegrator2DFactory {
 public:
  static PointcloudIntegrator2D::Ptr create(
      const std::string& integrator_type_name,
      VolumetricDataStructure2D::Ptr occupancy_map,
      std::optional<PointcloudIntegrator2DType> default_integrator_type =
          std::nullopt);

  static PointcloudIntegrator2D::Ptr create(
      PointcloudIntegrator2DType integrator_type,
      VolumetricDataStructure2D::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_2D_FACTORY_H_
