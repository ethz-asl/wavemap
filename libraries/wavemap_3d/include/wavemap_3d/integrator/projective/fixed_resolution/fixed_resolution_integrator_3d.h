#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_3D_H_

#include <utility>

#include <wavemap_common/data_structure/aabb.h>

#include "wavemap_3d/integrator/projective/range_image_2d.h"
#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
class FixedResolutionIntegrator3D : public ScanwiseIntegrator3D {
 public:
  using ScanwiseIntegrator3D::ScanwiseIntegrator3D;

  void integratePointcloud(const PosedPointcloud<Point3D>& pointcloud) override;

  static std::pair<RangeImage2D, AABB<Point3D>> computeRangeImageAndAABB(
      const PosedPointcloud<Point3D>& pointcloud,
      const SphericalProjector& spherical_projector);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_3D_H_
