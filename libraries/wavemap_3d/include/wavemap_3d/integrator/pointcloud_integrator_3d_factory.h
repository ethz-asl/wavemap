#ifndef WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_3D_FACTORY_H_
#define WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_3D_FACTORY_H_

#include <string>

#include <wavemap_common/integrator/projection_model/image_2d/image_2d_projection_model.h>
#include <wavemap_common/utils/config_utils.h>
#include <wavemap_common/utils/factory_utils.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/integrator/pointcloud_integrator_3d.h"

namespace wavemap {
struct PointcloudIntegrator3DType : TypeSelector<PointcloudIntegrator3DType> {
  using TypeSelector<PointcloudIntegrator3DType>::TypeSelector;

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

class PointcloudIntegrator3DFactory {
 public:
  static PointcloudIntegrator3D::Ptr create(
      const param::Map& params, VolumetricDataStructure3D::Ptr occupancy_map,
      std::optional<PointcloudIntegrator3DType> default_integrator_type =
          std::nullopt);

  static PointcloudIntegrator3D::Ptr create(
      PointcloudIntegrator3DType integrator_type, const param::Map& params,
      VolumetricDataStructure3D::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_3D_FACTORY_H_
