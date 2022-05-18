#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/coarse_to_fine_integrator.h"

#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/scalar_quadtree.h"
#include "wavemap_2d/iterator/grid_iterator.h"

namespace wavemap_2d {
void CoarseToFineIntegrator::integratePointcloud(
    const PosedPointcloud<>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  // TODO(victorr): Make this configurable
  // TODO(victorr): Avoid reallocating the range image (zero and reuse instead)
  const RangeImage range_image =
      computeRangeImage(pointcloud, -M_PI_2f32, M_PI_2f32, pointcloud.size());
  RangeImageIntersector range_image_intersector(range_image);

  // Get a pointer to the underlying specialized quadtree data structure
  //  using QuadtreeType = ScalarQuadtree<UnboundedOccupancyCell>;
  auto occupancy_map = dynamic_cast<VolumetricQuadtree*>(occupancy_map_.get());
  if (!occupancy_map) {
    LOG(FATAL) << "Coarse to fine integrator can only be used with "
                  "quadtree-based volumetric data structures.";
    return;
  }

  // Recursively update all relevant cells
  const QuadtreeIndex::Element max_depth = occupancy_map->getMaxDepth();
  const Transformation T_CW = pointcloud.getPose().inverse();
  const Point& t_W_C = pointcloud.getOrigin();
  const FloatingPoint root_node_width = occupancy_map->getRootNodeWidth();

  std::stack<QuadtreeIndex> stack;
  stack.emplace(QuadtreeIndex{});
  while (!stack.empty()) {
    const auto current_node = std::move(stack.top());
    stack.pop();

    const AABB<Point> W_cell_aabb =
        convert::nodeIndexToAABB(current_node, root_node_width);
    const AABB<Point>::Corners C_cell_corners =
        T_CW.transformVectorized(W_cell_aabb.corners());

    const RangeImageIntersector::IntersectionType intersection_type =
        range_image_intersector.determineIntersectionType(
            range_image, t_W_C, W_cell_aabb, C_cell_corners);
    if (intersection_type ==
        RangeImageIntersector::IntersectionType::kFullyUnknown) {
      continue;
    }

    const FloatingPoint node_width = W_cell_aabb.width<0>();
    const Point W_node_center =
        W_cell_aabb.min + Vector::Constant(node_width / 2.f);
    const Point C_node_center = T_CW * W_node_center;
    const FloatingPoint node_center_distance = C_node_center.norm();
    constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.41421356237f / 2.f;
    const FloatingPoint bounding_sphere_radius =
        kUnitCubeHalfDiagonal * node_width;
    if (max_depth <= current_node.depth ||
        isApproximationErrorAcceptable(intersection_type, node_center_distance,
                                       bounding_sphere_radius)) {
      FloatingPoint sample = computeUpdateForCell(range_image, C_node_center);
      occupancy_map->addToCellValue(current_node, sample);
      continue;
    }

    for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < QuadtreeIndex::kNumChildren;
         ++relative_child_idx) {
      stack.emplace(current_node.computeChildIndex(relative_child_idx));
    }
  }
}

RangeImage CoarseToFineIntegrator::computeRangeImage(
    const PosedPointcloud<>& pointcloud, FloatingPoint min_angle,
    FloatingPoint max_angle, Eigen::Index num_beams) {
  RangeImage range_image(min_angle, max_angle, num_beams);

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
  }

  return range_image;
}
}  // namespace wavemap_2d
