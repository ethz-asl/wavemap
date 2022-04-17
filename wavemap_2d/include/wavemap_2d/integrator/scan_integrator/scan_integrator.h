#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_

#include <algorithm>
#include <limits>
#include <utility>

#include "wavemap_2d/data_structure/generic/range_image.h"
#include "wavemap_2d/integrator/pointcloud_integrator.h"

namespace wavemap_2d {
class ScanIntegrator : public PointcloudIntegrator {
 public:
  explicit ScanIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override {
    if (!isPointcloudValid(pointcloud)) {
      return;
    }

    // Compute the range image and the scan's AABB
    // TODO(victorr): Make this configurable
    RangeImage range_image(-M_PI_2f32, M_PI_2f32, pointcloud.size());
    FloatingPoint range_max = 0.f;
    Point aabb_min = Point::Constant(std::numeric_limits<FloatingPoint>::max());
    Point aabb_max =
        Point::Constant(std::numeric_limits<FloatingPoint>::lowest());
    for (const auto& C_point : pointcloud.getPointsLocal()) {
      // Filter out noisy points and compute point's range
      if (C_point.hasNaN()) {
        LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                     << C_point;
        continue;
      }
      const FloatingPoint range = C_point.norm();
      if (1e3 < range) {
        LOG(INFO) << "Skipping measurement with suspicious length: " << range;
        continue;
      }

      // Add the point to the range image
      const RangeImageIndex range_image_index =
          range_image.bearingToNearestIndex(C_point);
      range_image[range_image_index] = range;

      // Update the AABB (in world frame)
      Point C_point_truncated = C_point;
      if (BeamModel::kRangeMax < range) {
        C_point_truncated *= BeamModel::kRangeMax / range;
        range_max = std::max(range_max, BeamModel::kRangeMax);
      } else {
        range_max = std::max(range_max, range);
      }
      const Point W_point_truncated = pointcloud.getPose() * C_point_truncated;
      aabb_min = aabb_min.cwiseMin(W_point_truncated);
      aabb_max = aabb_max.cwiseMax(W_point_truncated);
    }

    // Pad the aabb to account for the beam uncertainties
    const FloatingPoint max_lateral_component =
        std::max(std::sin(BeamModel::kAngleThresh) *
                     (range_max + BeamModel::kRangeDeltaThresh),
                 BeamModel::kRangeDeltaThresh);
    aabb_min -= Vector::Constant(max_lateral_component);
    aabb_max += Vector::Constant(max_lateral_component);

    // Compute the min and max map indices that could be affected by the cloud
    const FloatingPoint resolution = occupancy_map_->getResolution();
    const FloatingPoint resolution_inv = 1.f / resolution;
    const Index aabb_min_index =
        computeFloorIndexForPoint(aabb_min, resolution_inv);
    const Index aabb_max_index =
        computeCeilIndexForPoint(aabb_max, resolution_inv);

    // Iterate over all the cells in the AABB and update the map when needed
    for (const Index& index : Grid(aabb_min_index, aabb_max_index)) {
      const Point W_cell_center = computeCenterFromIndex(index, resolution);
      const Point C_cell_center =
          pointcloud.getPose().inverse() * W_cell_center;

      const FloatingPoint cell_to_sensor_distance = C_cell_center.norm();
      const FloatingPoint cell_azimuth_angle =
          RangeImage::bearingToAngle(C_cell_center);

      const auto first_idx =
          std::max(0l, range_image.angleToCeilIndex(cell_azimuth_angle -
                                                    BeamModel::kAngleThresh));
      const auto last_idx =
          std::min(range_image.getNBeams() - 1,
                   range_image.angleToFloorIndex(cell_azimuth_angle +
                                                 BeamModel::kAngleThresh));
      FloatingPoint total_update = 0.f;
      for (RangeImageIndex idx = first_idx; idx <= last_idx; ++idx) {
        const FloatingPoint measured_distance = range_image[idx];
        if (BeamModel::kRangeMax < cell_to_sensor_distance) {
          continue;
        }
        if (cell_to_sensor_distance < kEpsilon ||
            measured_distance + BeamModel::kRangeDeltaThresh <
                cell_to_sensor_distance) {
          continue;
        }

        const FloatingPoint beam_azimuth_angle = range_image.indexToAngle(idx);
        const FloatingPoint cell_to_beam_angle =
            std::abs(cell_azimuth_angle - beam_azimuth_angle);
        if (BeamModel::kAngleThresh < cell_to_beam_angle) {
          continue;
        }

        total_update += BeamModel::computeUpdate(
            cell_to_sensor_distance, cell_to_beam_angle, measured_distance);
      }
      if (kEpsilon < std::abs(total_update)) {
        occupancy_map_->addToCellValue(index, total_update);
      }
    }
  }
};
}  // namespace wavemap_2d
#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_
