#include "wavemap_2d/pointcloud_integrator.h"

namespace wavemap_2d {
void PointcloudIntegrator::integratePointcloud(
    const PosedPointcloud& pointcloud) {
  beam_model_.setStartPoint(pointcloud.getOrigin());
  for (const auto& point : pointcloud.getPointsGlobal()) {
    beam_model_.setEndPoint(point);
    if (40.f < beam_model_.getLength()) {
      continue;
    }

    aabb_min_ = aabb_min_.cwiseMin(point);
    aabb_max_ = aabb_max_.cwiseMax(point);

    const Index bottom_left_idx = beam_model_.getBottomLeftUpdateIndex();
    const Index top_right_idx = beam_model_.getTopRightUpdateIndex();
    for (Index index = bottom_left_idx; index.x() <= top_right_idx.x();
         ++index.x()) {
      for (index.y() = bottom_left_idx.y(); index.y() <= top_right_idx.y();
           ++index.y()) {
        const FloatingPoint update = beam_model_.computeUpdateAt(index);
        occupancy_map_->updateCell(index, update);
      }
    }
  }
}
}  // namespace wavemap_2d
