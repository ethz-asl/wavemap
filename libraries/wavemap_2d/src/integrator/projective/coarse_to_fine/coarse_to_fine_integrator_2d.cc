#include "wavemap_2d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_2d.h"

#include <stack>

#include <wavemap_common/indexing/ndtree_index.h>

namespace wavemap {
CoarseToFineIntegrator2D::CoarseToFineIntegrator2D(
    const PointcloudIntegratorConfig& config,
    CircularProjector projection_model,
    ContinuousVolumetricLogOdds<2> measurement_model,
    VolumetricDataStructure2D::Ptr occupancy_map)
    : ScanwiseIntegrator2D(config, std::move(projection_model),
                           std::move(measurement_model),
                           std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()) {
  // Get a pointer to the underlying specialized quadtree data structure
  volumetric_quadtree_ =
      std::dynamic_pointer_cast<VolumetricQuadtreeInterface>(occupancy_map_);
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
  updateRangeImage(pointcloud, *posed_range_image_);
  range_image_intersector_ = std::make_shared<RangeImage1DIntersector>(
      posed_range_image_, config_.max_range,
      measurement_model_.getAngleThreshold(),
      measurement_model_.getRangeThresholdInFrontOfSurface(),
      measurement_model_.getRangeThresholdBehindSurface());

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
        range_image_intersector_->determineIntersectionType(
            pointcloud.getPose(), W_cell_aabb, projection_model_);
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
      const FloatingPoint sample = computeUpdate(C_node_center);
      if (kEpsilon < std::abs(sample)) {
        volumetric_quadtree_->addToCellValue(current_node, sample);
      }
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
