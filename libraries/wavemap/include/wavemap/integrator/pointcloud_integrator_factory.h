#ifndef WAVEMAP_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_
#define WAVEMAP_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_

#include <string>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/pointcloud_integrator.h"
#include "wavemap/integrator/projection_model/image_2d/image_2d_projection_model.h"
#include "wavemap/utils/config_utils.h"
#include "wavemap/utils/factory_utils.h"

namespace wavemap {
struct PointcloudIntegratorType : TypeSelector<PointcloudIntegratorType> {
  using TypeSelector<PointcloudIntegratorType>::TypeSelector;

  enum Id : TypeId {
    kSingleRayIntegrator,
    kFixedResolutionScanIntegrator,
    kCoarseToFineScanIntegrator,
    kWaveletScanIntegrator
  };

  static constexpr std::array names = {
      "single_ray_integrator", "fixed_resolution_integrator",
      "coarse_to_fine_integrator", "coarse_to_fine_wavelet_integrator"};
};

struct Image2DProjectionModelType : TypeSelector<Image2DProjectionModelType> {
  using TypeSelector<Image2DProjectionModelType>::TypeSelector;

  enum Id : TypeId {
    kSphericalProjector,
    kOusterProjector,
    kPinholeCameraProjector
  };

  static constexpr std::array names = {
      "spherical_projector", "ouster_projector", "pinhole_camera_projector"};
};

class PointcloudIntegratorFactory {
 public:
  static PointcloudIntegrator::Ptr create(
      const param::Map& params, VolumetricDataStructureBase::Ptr occupancy_map,
      std::optional<PointcloudIntegratorType> default_integrator_type =
          std::nullopt);

  static PointcloudIntegrator::Ptr create(
      PointcloudIntegratorType integrator_type, const param::Map& params,
      VolumetricDataStructureBase::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_POINTCLOUD_INTEGRATOR_FACTORY_H_
