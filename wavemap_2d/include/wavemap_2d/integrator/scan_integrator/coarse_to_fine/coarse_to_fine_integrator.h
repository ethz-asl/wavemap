#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_

#include <algorithm>
#include <utility>

#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/range_image_intersector.h"
#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap_2d {
class CoarseToFineIntegrator : public PointcloudIntegrator {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  explicit CoarseToFineIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override;

  static RangeImage computeRangeImage(const PosedPointcloud<>& pointcloud,
                                      FloatingPoint min_angle,
                                      FloatingPoint max_angle,
                                      Eigen::Index num_beams);

 private:
  static constexpr FloatingPoint kMaxGradientOverRangeFullyInside =
      BeamModel::kScaling * 572.957795130823f;
  static constexpr FloatingPoint kMaxGradientOnBoundary =
      BeamModel::kScaling * 14.9999999999997f;

  static bool isApproximationErrorAcceptable(
      RangeImageIntersector::IntersectionType intersection_type,
      FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius);

  static FloatingPoint computeUpdateForCell(const RangeImage& range_image,
                                            FloatingPoint d_C_cell,
                                            FloatingPoint azimuth_angle_C_cell);
};
}  // namespace wavemap_2d

#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/impl/coarse_to_fine_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_
