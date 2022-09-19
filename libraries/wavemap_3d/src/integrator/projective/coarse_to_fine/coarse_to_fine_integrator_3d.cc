#include "wavemap_3d/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_3d.h"

#include <stack>

#include <wavemap_common/indexing/ndtree_index.h>

#include "wavemap_3d/integrator/projective/coarse_to_fine/range_image_2d_intersector.h"

namespace wavemap {
CoarseToFineIntegrator3D::CoarseToFineIntegrator3D(
    VolumetricDataStructure3D::Ptr occupancy_map)
    : ScanwiseIntegrator3D(std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()),
      posed_range_image_(
          std::make_shared<PosedRangeImage2D>(spherical_projector_)) {
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
  posed_range_image_->importPointcloud(pointcloud, spherical_projector_);
  const RangeImage2DIntersector range_image_intersector(posed_range_image_);

  // Recursively update all relevant cells
  std::stack<OctreeIndex> stack;
  for (const OctreeIndex& node_index :
       volumetric_octree_->getFirstChildIndices()) {
    stack.emplace(node_index);
  }
  while (!stack.empty()) {
    const auto current_node = std::move(stack.top());
    stack.pop();

    const AABB<Point3D> W_cell_aabb =
        convert::nodeIndexToAABB(current_node, min_cell_width_);
    const IntersectionType intersection_type =
        range_image_intersector.determineIntersectionType(
            pointcloud.getPose(), W_cell_aabb, spherical_projector_);
    if (intersection_type == IntersectionType::kFullyUnknown) {
      continue;
    }

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
      const Vector2D spherical_C_cell =
          SphericalProjector::bearingToSpherical(C_node_center);
      const FloatingPoint sample =
          computeUpdate(*posed_range_image_, d_C_cell, spherical_C_cell);
      if (kEpsilon < std::abs(sample)) {
        volumetric_octree_->addToCellValue(current_node, sample);
      }
      continue;
    }

    for (OctreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      stack.emplace(current_node.computeChildIndex(relative_child_idx));
    }
  }
}
}  // namespace wavemap
