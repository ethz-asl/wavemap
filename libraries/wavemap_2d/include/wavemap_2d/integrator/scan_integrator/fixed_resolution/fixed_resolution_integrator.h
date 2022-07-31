#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_

#include <utility>

#include <wavemap_common/data_structure/aabb.h>

#include "wavemap_2d/integrator/scan_integrator/range_image.h"
#include "wavemap_2d/integrator/scan_integrator/scan_integrator.h"

namespace wavemap {
class FixedResolutionIntegrator : public ScanIntegrator {
 public:
  using ScanIntegrator::ScanIntegrator;

  void integratePointcloud(
      const PosedPointcloud<Point2D, Transformation2D>& pointcloud) override;

  static std::pair<RangeImage, AABB<Point2D>> computeRangeImageAndAABB(
      const PosedPointcloud<Point2D, Transformation2D>& pointcloud,
      const CircleProjector& circle_projector);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
