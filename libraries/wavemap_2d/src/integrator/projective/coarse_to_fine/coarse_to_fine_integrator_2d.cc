#include "wavemap_2d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_2d.h"

#include <stack>

#include <wavemap_common/indexing/ndtree_index.h>

namespace wavemap {
CoarseToFineIntegrator2D::CoarseToFineIntegrator2D(
    VolumetricDataStructure2D::Ptr occupancy_map)
    : ScanwiseIntegrator2D(std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()),
      posed_range_image_(
          std::make_shared<PosedRangeImage1D>(circular_projector_)) {
  // Get a pointer to the underlying specialized quadtree data structure
  volumetric_quadtree_ =
      dynamic_cast<VolumetricQuadtreeInterface*>(occupancy_map_.get());
  CHECK(volumetric_quadtree_)
      << "Coarse to fine integrator can only be used with quadtree-based "
         "volumetric data structures.";
}

void CoarseToFineIntegrator2D::integratePointcloud(
    const PosedPointcloud<Point2D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  posed_range_image_->importPointcloud(pointcloud, circular_projector_);
  const RangeImage1DIntersector range_image_intersector(posed_range_image_);

  // Recursively update all relevant cells
  std::stack<QuadtreeIndex> stack;
  for (const QuadtreeIndex& node_index :
       volumetric_quadtree_->getFirstChildIndices()) {
    stack.emplace(node_index);
  }
  while (!stack.empty()) {
    const auto current_node = std::move(stack.top());
    stack.pop();

    const AABB<Point2D> W_cell_aabb =
        convert::nodeIndexToAABB(current_node, min_cell_width_);
    const IntersectionType intersection_type =
        range_image_intersector.determineIntersectionType(
            pointcloud.getPose(), W_cell_aabb, circular_projector_);
    if (intersection_type == IntersectionType::kFullyUnknown) {
      continue;
    }

    const FloatingPoint node_width = W_cell_aabb.width<0>();
    const Point2D W_node_center =
        W_cell_aabb.min + Vector2D::Constant(node_width / 2.f);
    const Point2D C_node_center =
        posed_range_image_->getPoseInverse() * W_node_center;
    const FloatingPoint d_C_cell = C_node_center.norm();
    const FloatingPoint bounding_sphere_radius =
        kUnitSquareHalfDiagonal * node_width;
    if (current_node.height == 0 ||
        isApproximationErrorAcceptable(intersection_type, d_C_cell,
                                       bounding_sphere_radius)) {
      const FloatingPoint angle_C_cell =
          CircularProjector::bearingToAngle(C_node_center);
      const FloatingPoint sample =
          computeUpdate(*posed_range_image_, d_C_cell, angle_C_cell);
      volumetric_quadtree_->addToCellValue(current_node, sample);
      continue;
    }

    for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < QuadtreeIndex::kNumChildren;
         ++relative_child_idx) {
      stack.emplace(current_node.computeChildIndex(relative_child_idx));
    }
  }
}
}  // namespace wavemap
