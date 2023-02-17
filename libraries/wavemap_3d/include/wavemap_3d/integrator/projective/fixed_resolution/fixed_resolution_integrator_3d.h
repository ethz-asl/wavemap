#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_3D_H_

#include <utility>

#include <wavemap/data_structure/aabb.h>

#include "wavemap_3d/integrator/projective/range_image_2d.h"
#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
class FixedResolutionIntegrator3D : public ScanwiseIntegrator3D {
 public:
  using ScanwiseIntegrator3D::ScanwiseIntegrator3D;

 private:
  AABB<Point3D> aabb_;

  void importPointcloud(
      const PosedPointcloud<wavemap::Point3D>& pointcloud) override;
  void importRangeImage(
      const wavemap::PosedRangeImage2D& range_image_input) override;

  void updateMap() override;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_3D_H_
