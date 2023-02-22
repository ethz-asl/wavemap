#include "wavemap/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.h"

#include <stack>

#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
void CoarseToFineIntegrator::updateMap() {
  // Update the range image intersector
  range_image_intersector_ = std::make_shared<RangeImageIntersector>(
      posed_range_image_, projection_model_, *measurement_model_,
      config_.min_range, config_.max_range);

  // Recursively update all relevant cells
  std::stack<OctreeIndex> stack;
  for (const OctreeIndex& node_index : occupancy_map_->getFirstChildIndices()) {
    stack.emplace(node_index);
  }
  while (!stack.empty()) {
    auto current_node = std::move(stack.top());
    stack.pop();

    // If we're at the leaf level, directly update the node
    if (current_node.height == 0) {
      const Point3D W_node_center =
          convert::nodeIndexToCenterPoint(current_node, min_cell_width_);
      const Point3D C_node_center =
          posed_range_image_->getPoseInverse() * W_node_center;
      const FloatingPoint sample = computeUpdate(C_node_center);
      if (kEpsilon < std::abs(sample)) {
        occupancy_map_->addToCellValue(current_node, sample);
      }
      continue;
    }

    // Otherwise, test whether the current node is fully occupied;
    // free or unknown; or fully unknown
    const AABB<Point3D> W_cell_aabb =
        convert::nodeIndexToAABB(current_node, min_cell_width_);
    const UpdateType update_type =
        range_image_intersector_->determineUpdateType(
            W_cell_aabb, posed_range_image_->getRotationMatrixInverse(),
            posed_range_image_->getPose().getPosition());

    // If we're fully in unknown space,
    // there's no need to evaluate this node or its children
    if (update_type == UpdateType::kFullyUnobserved) {
      continue;
    }

    // Test if the worst-case error for the intersection type at the current
    // resolution falls within the acceptable approximation error
    const FloatingPoint node_width = W_cell_aabb.width<0>();
    const Point3D W_node_center =
        W_cell_aabb.min + Vector3D::Constant(node_width / 2.f);
    const Point3D C_node_center =
        posed_range_image_->getPoseInverse() * W_node_center;
    const FloatingPoint d_C_cell =
        projection_model_->cartesianToSensorZ(C_node_center);
    const FloatingPoint bounding_sphere_radius =
        kUnitCubeHalfDiagonal * node_width;
    if (current_node.height == 0 ||
        measurement_model_->computeWorstCaseApproximationError(
            update_type, d_C_cell, bounding_sphere_radius) <
            config_.termination_update_error) {
      const FloatingPoint sample = computeUpdate(C_node_center);
      if (kEpsilon < std::abs(sample)) {
        occupancy_map_->addToCellValue(current_node, sample);
      }
      continue;
    }

    // Since the approximation error would still be too big, refine
    for (OctreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      stack.emplace(current_node.computeChildIndex(relative_child_idx));
    }
  }
}
}  // namespace wavemap
