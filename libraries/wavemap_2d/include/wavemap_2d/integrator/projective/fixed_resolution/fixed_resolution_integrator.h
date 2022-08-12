#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_

#include <utility>

#include <wavemap_common/data_structure/aabb.h>

#include "wavemap_2d/integrator/projective/range_image_1d.h"
#include "wavemap_2d/integrator/projective/scanwise_integrator_2d.h"

namespace wavemap {
class FixedResolutionIntegrator : public ScanwiseIntegrator2D {
 public:
  using ScanwiseIntegrator2D::ScanwiseIntegrator2D;

  void integratePointcloud(const PosedPointcloud<Point2D>& pointcloud) override;

  static std::pair<RangeImage1D, AABB<Point2D>> computeRangeImageAndAABB(
      const PosedPointcloud<Point2D>& pointcloud,
      const CircularProjector& circular_projector);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
