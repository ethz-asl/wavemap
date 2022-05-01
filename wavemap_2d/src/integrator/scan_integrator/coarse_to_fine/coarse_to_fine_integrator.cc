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
  RangeImage range_image(-M_PI_2f32, M_PI_2f32, pointcloud.size());
  computeRangeImage(pointcloud, range_image);
  RangeImageIntersector range_image_intersector(range_image);

  // Recursively update all relevant cells
  const FloatingPoint resolution = occupancy_map_->getResolution();
  ScalarQuadtree<UnboundedOccupancyCell> test_quadtree(resolution);
  std::function<void(const QuadtreeIndex&)> recursive_fn =
      [&](const QuadtreeIndex& node_idx) {
        const FloatingPoint node_width =
            test_quadtree.computeNodeWidthAtDepth(node_idx.depth);
        const Point W_node_center =
            computeCenterFromIndex(node_idx.position, node_width);
        const Point C_node_center =
            pointcloud.getPose().inverse() * W_node_center;
        const FloatingPoint node_center_distance = C_node_center.norm();

        constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.73205080757f / 2.f;
        const FloatingPoint bounding_sphere_radius =
            kUnitCubeHalfDiagonal * node_width;
        const RangeImageIntersector::IntersectionType intersection_type =
            range_image_intersector.determineIntersectionType(
                range_image, C_node_center, node_center_distance,
                bounding_sphere_radius);

        if (intersection_type ==
            RangeImageIntersector::IntersectionType::kFullyOutside) {
          return;
        }

        if (kMaxDepth <= node_idx.depth ||
            computeMaxApproximationError(
                intersection_type, node_center_distance,
                bounding_sphere_radius) <= kMaxAcceptableError) {
          //          LOG(INFO) << "Sampling: " << node_idx.toString();
          FloatingPoint sample =
              computeUpdateForCell(range_image, C_node_center);
          test_quadtree.setCellValue(node_idx, sample);
          return;
        }

        for (const QuadtreeIndex& child_idx : node_idx.computeChildIndices()) {
          recursive_fn(child_idx);
        }
      };

  recursive_fn(QuadtreeIndex{});

  const Index min_index = test_quadtree.getMinIndex();
  const Index max_index = test_quadtree.getMaxIndex();
  for (const Index& index : Grid(min_index, max_index)) {
    const FloatingPoint update = test_quadtree.getCellValue(index);
    occupancy_map_->addToCellValue(index, update);
  }
  //  occupancy_map_->showImage(true, 1000);
}

void CoarseToFineIntegrator::computeRangeImage(
    const PosedPointcloud<>& pointcloud, RangeImage& range_image) {
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
}
}  // namespace wavemap_2d
