#ifndef WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_
#define WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_

#include <string>

#include <wavemap_common/utils/type_utils.h>

#include "wavemap_3d/integrator/pointcloud_integrator.h"

namespace wavemap {
enum class PointcloudIntegratorType : int { kSingleRayIntegrator };
constexpr std::array<const char*, 1> kPointcloudIntegratorTypeStrs = {
    "single_ray_integrator"};
std::string getPointcloudIntegratorTypeStr(
    PointcloudIntegratorType intersection_type) {
  return kPointcloudIntegratorTypeStrs[to_underlying(intersection_type)];
}

class PointcloudIntegratorFactory {
 public:
  static PointcloudIntegrator::Ptr create(
      const std::string& integrator_type_name,
      VolumetricDataStructure3D::Ptr occupancy_map,
      std::optional<PointcloudIntegratorType> default_integrator_type =
          std::nullopt);

  static PointcloudIntegrator::Ptr create(
      PointcloudIntegratorType integrator_type,
      VolumetricDataStructure3D::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_
