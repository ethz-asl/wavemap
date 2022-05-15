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
  auto occupancy_map = dynamic_cast<ScalarQuadtree<SaturatingOccupancyCell>*>(
      occupancy_map_.get());
  if (!occupancy_map) {
    LOG(FATAL) << "Coarse to fine integrator can only be used with quadtree "
                  "data structure.";
    return;
  }

  // Recursively update all relevant cells
  const QuadtreeIndex::Element max_depth = occupancy_map->getMaxDepth();
  const Transformation T_CW = pointcloud.getPose().inverse();
  const Point& t_W_C = pointcloud.getOrigin();
  std::function<void(const QuadtreeIndex&)> recursive_fn =
      [&](const QuadtreeIndex& node_idx) {
        const FloatingPoint node_width =
            occupancy_map->computeNodeWidthAtDepth(node_idx.depth);
        const Point W_node_bottom_left = computeNodeCenterFromNodeIndex(
            node_idx, occupancy_map->getRootNodeWidth());
        const Point W_node_center =
            W_node_bottom_left + Vector::Constant(node_width / 2);

        Eigen::Matrix<FloatingPoint, 2, 4> W_cell_corners =
            W_node_bottom_left.replicate<1, 4>();
        for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
          for (int dim_idx = 0; dim_idx < 2; ++dim_idx) {
            if (corner_idx & (0b1 << dim_idx)) {
              W_cell_corners(dim_idx, corner_idx) += node_width;
            }
          }
        }
        const AABB<Point> W_cell_aabb{W_cell_corners.col(0),
                                      W_cell_corners.col(3)};
        const Eigen::Matrix<FloatingPoint, 2, 4> C_cell_corners =
            T_CW.transformVectorized(W_cell_corners);
        const RangeImageIntersector::IntersectionType intersection_type =
            range_image_intersector.determineIntersectionType(
                range_image, t_W_C, W_cell_aabb, C_cell_corners);
        if (intersection_type ==
            RangeImageIntersector::IntersectionType::kFullyUnknown) {
          return;
        }

        const Point C_node_center = T_CW * W_node_center;
        const FloatingPoint node_center_distance = C_node_center.norm();
        constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.41421356237f / 2.f;
        const FloatingPoint bounding_sphere_radius =
            kUnitCubeHalfDiagonal * node_width;
        if (max_depth <= node_idx.depth ||
            isApproximationErrorAcceptable(intersection_type,
                                           node_center_distance,
                                           bounding_sphere_radius)) {
          FloatingPoint sample =
              computeUpdateForCell(range_image, C_node_center);
          occupancy_map->addToCellValue(node_idx, sample);
          return;
        }

        for (const QuadtreeIndex& child_idx : node_idx.computeChildIndices()) {
          recursive_fn(child_idx);
        }
      };

  recursive_fn(QuadtreeIndex{});
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
