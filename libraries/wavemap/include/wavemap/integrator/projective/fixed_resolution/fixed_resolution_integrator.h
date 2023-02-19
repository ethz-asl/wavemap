#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_

#include <utility>

#include "wavemap/data_structure/aabb.h"
#include "wavemap/integrator/projective/range_image_2d.h"
#include "wavemap/integrator/projective/scanwise_integrator.h"

namespace wavemap {
class FixedResolutionIntegrator : public ScanwiseIntegrator {
 public:
  using ScanwiseIntegrator::ScanwiseIntegrator;

 private:
  AABB<Point3D> aabb_;

  void importPointcloud(
      const PosedPointcloud<wavemap::Point3D>& pointcloud) override;
  void importRangeImage(
      const wavemap::PosedRangeImage2D& range_image_input) override;

  void updateMap() override;
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
