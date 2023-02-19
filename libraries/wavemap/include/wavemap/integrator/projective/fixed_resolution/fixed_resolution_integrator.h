#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_

#include <utility>

#include "wavemap/data_structure/aabb.h"
#include "wavemap/data_structure/image.h"
#include "wavemap/integrator/projective/scanwise_integrator.h"

namespace wavemap {
class FixedResolutionIntegrator : public ScanwiseIntegrator {
 public:
  using ScanwiseIntegrator::ScanwiseIntegrator;

 private:
  AABB<Point3D> aabb_;

  void importPointcloud(const PosedPointcloud<Point3D>& pointcloud) override;
  void importRangeImage(const PosedImage<>& range_image_input) override;

  void updateMap() override;
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
