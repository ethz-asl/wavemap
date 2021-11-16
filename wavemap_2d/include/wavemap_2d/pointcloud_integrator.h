#ifndef WAVEMAP_2D_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_2D_POINTCLOUD_INTEGRATOR_H_

#include <limits>
#include <utility>

#include "wavemap_2d/beam_model.h"
#include "wavemap_2d/common.h"
#include "wavemap_2d/occupancy_map.h"
#include "wavemap_2d/pointcloud.h"

namespace wavemap_2d {
class PointcloudIntegrator {
 public:
  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(OccupancyMap::Ptr occupancy_map)
      : occupancy_map_(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud& pointcloud) {
    beam_model_.setStartPoint(pointcloud.getOrigin());
    for (const auto& point : pointcloud.getPointsGlobal()) {
      beam_model_.setEndPoint(point);

      aabb_min_ = aabb_min_.cwiseMin(point);
      aabb_max_ = aabb_max_.cwiseMax(point);

      constexpr FloatingPoint resolution = 1.f;  // m
      constexpr FloatingPoint resolution_inv = 1.f / resolution;
      const Index index =
          (point * resolution_inv).array().round().cast<IndexElement>();

      constexpr FloatingPoint kUpdate = 1.f;
      occupancy_map_->updateCell(index, kUpdate);
    }
  }

  void printAabbBounds() const {
    LOG(INFO) << "AABB min:\n" << aabb_min_ << "\nmax:\n" << aabb_max_;
  }
  void printSize() const { LOG(INFO) << "Size:\n" << occupancy_map_->size(); }

 protected:
  Point aabb_min_ = Point::Constant(std::numeric_limits<FloatingPoint>::max());
  Point aabb_max_ =
      Point::Constant(std::numeric_limits<FloatingPoint>::lowest());

  OccupancyMap::Ptr occupancy_map_;

  BeamModel beam_model_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_POINTCLOUD_INTEGRATOR_H_
