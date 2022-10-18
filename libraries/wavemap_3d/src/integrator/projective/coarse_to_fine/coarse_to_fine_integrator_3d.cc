#include "wavemap_3d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_3d.h"

#include <stack>

#include <wavemap_common/indexing/ndtree_index.h>

namespace wavemap {
CoarseToFineIntegrator3D::CoarseToFineIntegrator3D(
    const PointcloudIntegratorConfig& config,
    SphericalProjector projection_model,
    ContinuousVolumetricLogOdds<3> measurement_model,
    VolumetricDataStructure3D::Ptr occupancy_map)
    : ScanwiseIntegrator3D(config, std::move(projection_model),
                           std::move(measurement_model),
                           std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()) {
  // Get a pointer to the underlying specialized octree data structure
  volumetric_octree_ =
      dynamic_cast<VolumetricOctreeInterface*>(occupancy_map_.get());
  CHECK(volumetric_octree_)
      << "Coarse to fine integrator can only be used with octree-based "
         "volumetric data structures.";
}

void CoarseToFineIntegrator3D::integratePointcloud(
    const PosedPointcloud<Point3D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  updateRangeImage(pointcloud, *posed_range_image_, bearing_image_);
  range_image_intersector_ = std::make_shared<RangeImage2DIntersector>(
      posed_range_image_, projection_model_, config_.min_range,
      config_.max_range, measurement_model_.getAngleThreshold(),
      measurement_model_.getRangeThresholdInFrontOfSurface(),
      measurement_model_.getRangeThresholdBehindSurface());

  // Recursively update all relevant cells
  std::stack<std::pair<OctreeIndex, RangeImage2DIntersector::Cache>> stack;
  for (const OctreeIndex& node_index :
       volumetric_octree_->getFirstChildIndices()) {
    stack.emplace(node_index, RangeImage2DIntersector::Cache{});
  }
  while (!stack.empty()) {
    auto [current_node, cache] = std::move(stack.top());
    stack.pop();

    // If we're at the leaf level, directly update the node
    if (current_node.height == 0) {
      const Point3D W_node_center =
          convert::nodeIndexToCenterPoint(current_node, min_cell_width_);
      const Point3D C_node_center =
          posed_range_image_->getPoseInverse() * W_node_center;
      const FloatingPoint sample = computeUpdate(C_node_center);
      if (kEpsilon < std::abs(sample)) {
        volumetric_octree_->addToCellValue(current_node, sample);
      }
      continue;
    }

    // Otherwise, test whether the current node is fully occupied;
    // free or unknown; or fully unknown
    const AABB<Point3D> W_cell_aabb =
        convert::nodeIndexToAABB(current_node, min_cell_width_);
    const IntersectionType intersection_type =
        range_image_intersector_->determineIntersectionType(
            pointcloud.getPose(), W_cell_aabb, cache);

    // If we're fully in unknown space,
    // there's no need to evaluate this node or its children
    if (intersection_type == IntersectionType::kFullyUnknown) {
      continue;
    }

    // Test if the worst-case error for the intersection type at the current
    // resolution falls within the acceptable approximation error
    const FloatingPoint node_width = W_cell_aabb.width<0>();
    const Point3D W_node_center =
        W_cell_aabb.min + Vector3D::Constant(node_width / 2.f);
    const Point3D C_node_center =
        posed_range_image_->getPoseInverse() * W_node_center;
    const FloatingPoint d_C_cell = C_node_center.norm();
    const FloatingPoint bounding_sphere_radius =
        kUnitCubeHalfDiagonal * node_width;
    if (current_node.height == 0 ||
        isApproximationErrorAcceptable(intersection_type, d_C_cell,
                                       bounding_sphere_radius)) {
      const FloatingPoint sample = computeUpdate(C_node_center);
      if (kEpsilon < std::abs(sample)) {
        volumetric_octree_->addToCellValue(current_node, sample);
      }
      continue;
    }

    // Since the approximation error would still be too big, refine
    for (OctreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      stack.emplace(current_node.computeChildIndex(relative_child_idx), cache);
    }
  }
}
}  // namespace wavemap
