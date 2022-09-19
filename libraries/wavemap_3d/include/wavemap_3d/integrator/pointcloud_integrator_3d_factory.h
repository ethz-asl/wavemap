#ifndef WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_3D_FACTORY_H_
#define WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_3D_FACTORY_H_

#include <string>

#include <wavemap_common/utils/type_utils.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"

namespace wavemap {
enum class PointcloudIntegrator3DType : int {
  kSingleRayIntegrator,
  kFixedResolutionScanIntegrator,
  kCoarseToFineScanIntegrator,
  kWaveletScanIntegrator
};
constexpr std::array kPointcloudIntegrator3DTypeStrs = {
    "single_ray_integrator", "fixed_resolution_scan_integrator",
    "coarse_to_fine_scan_integrator", "wavelet_scan_integrator"};
std::string getPointcloudIntegrator3DTypeStr(
    PointcloudIntegrator3DType intersection_type) {
  return kPointcloudIntegrator3DTypeStrs[to_underlying(intersection_type)];
}

class PointcloudIntegrator3DFactory {
 public:
  static PointcloudIntegrator3D::Ptr create(
      const std::string& integrator_type_name,
      VolumetricDataStructure3D::Ptr occupancy_map,
      std::optional<PointcloudIntegrator3DType> default_integrator_type =
          std::nullopt);

  static PointcloudIntegrator3D::Ptr create(
      PointcloudIntegrator3DType integrator_type,
      VolumetricDataStructure3D::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_3D_FACTORY_H_
