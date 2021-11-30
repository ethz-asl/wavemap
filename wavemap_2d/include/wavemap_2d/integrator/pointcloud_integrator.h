#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <limits>
#include <utility>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/datastructure_base.h"
#include "wavemap_2d/integrator/beam_model.h"
#include "wavemap_2d/pointcloud.h"

namespace wavemap_2d {
class PointcloudIntegrator {
 public:
  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(DataStructureBase::Ptr occupancy_map)
      : occupancy_map_(CHECK_NOTNULL(occupancy_map)),
        beam_model_(occupancy_map->getResolution()) {}

  void integratePointcloud(const PosedPointcloud& pointcloud);

  void printAabbBounds() const {
    LOG(INFO) << "AABB min:\n" << aabb_min_ << "\nmax:\n" << aabb_max_;
  }

 protected:
  Point aabb_min_ = Point::Constant(std::numeric_limits<FloatingPoint>::max());
  Point aabb_max_ =
      Point::Constant(std::numeric_limits<FloatingPoint>::lowest());

  DataStructureBase::Ptr occupancy_map_;

  BeamModel beam_model_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
