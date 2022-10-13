#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_2D_FACTORY_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_2D_FACTORY_H_

#include <string>

#include <wavemap_common/utils/config_utils.h>
#include <wavemap_common/utils/factory_utils.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"
#include "wavemap_2d/integrator/pointcloud_integrator_2d.h"

namespace wavemap {
struct PointcloudIntegrator2DType : TypeSelector<PointcloudIntegrator2DType> {
  using TypeSelector<PointcloudIntegrator2DType>::TypeSelector;

  enum Id : TypeId {
    kSingleRayIntegrator,
    kSingleBeamIntegrator,
    kFixedResolutionScanIntegrator,
    kCoarseToFineScanIntegrator,
    kWaveletScanIntegrator
  };

  static constexpr std::array names = {
      "single_ray_integrator", "single_beam_integrator",
      "fixed_resolution_scan_integrator", "coarse_to_fine_scan_integrator",
      "wavelet_scan_integrator"};
};

class PointcloudIntegrator2DFactory {
 public:
  static PointcloudIntegrator2D::Ptr create(
      const param::Map& params, VolumetricDataStructure2D::Ptr occupancy_map,
      std::optional<PointcloudIntegrator2DType> default_integrator_type =
          std::nullopt);

  static PointcloudIntegrator2D::Ptr create(
      PointcloudIntegrator2DType integrator_type, const param::Map& params,
      VolumetricDataStructure2D::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_2D_FACTORY_H_
