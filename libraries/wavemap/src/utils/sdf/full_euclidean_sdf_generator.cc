#include "wavemap/utils/sdf/full_euclidean_sdf_generator.h"

#include <tracy/Tracy.hpp>

#include "wavemap/utils/iterate/grid_iterator.h"
#include "wavemap/utils/query/query_accelerator.h"

namespace wavemap {
HashedBlocks FullEuclideanSDFGenerator::generate(
    const HashedWaveletOctree& occupancy_map) const {
  ZoneScoped;
  // Initialize the SDF data structure
  const FloatingPoint min_cell_width = occupancy_map.getMinCellWidth();
  const Index3D uninitialized_parent =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  VectorDistanceField full_sdf(
      VectorDistance{uninitialized_parent, max_distance_});

  // Initialize the bucketed priority queue
  const int num_bins =
      static_cast<int>(std::ceil(max_distance_ / min_cell_width));
  BucketQueue<Index3D> open{num_bins, max_distance_};

  // Seed and propagate the SDF
  seed(occupancy_map, full_sdf, open);
  propagate(occupancy_map, full_sdf, open);

  // Copy into regular data structure
  const MapBaseConfig config{min_cell_width, 0.f, max_distance_};
  HashedBlocks sdf(config, max_distance_);
  full_sdf.forEachLeaf(
      [&sdf](const Index3D& cell_index, const VectorDistance& cell_value) {
        sdf.setCellValue(cell_index, cell_value.distance);
      });

  return sdf;
}

void FullEuclideanSDFGenerator::seed(const HashedWaveletOctree& occupancy_map,
                                     VectorDistanceField& sdf,
                                     BucketQueue<Index3D>& open_queue) const {
  ZoneScoped;
  // Create an occupancy query accelerator
  QueryAccelerator occupancy_query_accelerator{occupancy_map};

  // For all free cells that border an obstacle:
  // - initialize their SDF value, and
  // - add them to the open queue
  occupancy_map.forEachLeaf([this, &occupancy_query_accelerator, &sdf,
                             &open_queue,
                             min_cell_width = occupancy_map.getMinCellWidth()](
                                const OctreeIndex& node_index,
                                FloatingPoint node_occupancy) {
    // Only process obstacles
    if (!classifier_.is(node_occupancy, Occupancy::kOccupied)) {
      return;
    }

    // Span a grid at the highest resolution (=SDF resolution) that pads the
    // multi-resolution obstacle cell with 1 voxel in all directions
    const Index3D min_corner = convert::nodeIndexToMinCornerIndex(node_index);
    const Index3D max_corner = convert::nodeIndexToMaxCornerIndex(node_index);
    const Grid<3> grid{
        Grid<3>(min_corner - Index3D::Ones(), max_corner + Index3D::Ones())};

    // Iterate over the grid
    for (const Index3D& index : grid) {
      // Skip cells that are inside the occupied node (obstacle)
      // NOTE: Occupied cells (negative distances) are handled in the
      //       propagation stage.
      const Index3D nearest_inner_index =
          index.cwiseMax(min_corner).cwiseMin(max_corner);
      const bool voxel_is_inside = (index == nearest_inner_index);
      if (voxel_is_inside) {
        continue;
      }

      // Skip the cell if it is not free
      const FloatingPoint occupancy =
          occupancy_query_accelerator.getCellValue(index);
      if (!classifier_.is(occupancy, Occupancy::kFree)) {
        continue;
      }

      // Get the voxel's SDF value
      auto& [sdf_parent, sdf_value] = sdf.getOrAllocateValue(index);
      const bool sdf_uninitialized = sdf_parent == sdf.getDefaultValue().parent;

      // Update the voxel's SDF value
      const FloatingPoint distance_to_surface =
          minDistanceTo(index, nearest_inner_index, min_cell_width);
      if (distance_to_surface < sdf_value) {
        sdf_value = distance_to_surface;
        sdf_parent = nearest_inner_index;
      }

      // If the voxel is not yet in the open queue, add it
      if (sdf_uninitialized) {
        open_queue.push(distance_to_surface, index);
      }
    }
  });
}

void FullEuclideanSDFGenerator::propagate(
    const HashedWaveletOctree& occupancy_map, VectorDistanceField& sdf,
    BucketQueue<Index3D>& open_queue) const {
  ZoneScoped;
  // Create an occupancy query accelerator
  QueryAccelerator occupancy_query_accelerator{occupancy_map};

  // Precompute the neighbor distance offsets
  const FloatingPoint min_cell_width = occupancy_map.getMinCellWidth();
  const FloatingPoint half_max_neighbor_distance_offset =
      0.5f * std::sqrt(3.f) * min_cell_width + 1e-3f;

  // Propagate the distance
  while (!open_queue.empty()) {
    TracyPlot("QueueLength", static_cast<int64_t>(open_queue.size()));
    const Index3D index = open_queue.front();
    const auto& [sdf_parent, sdf_value] = sdf.getValueOrDefault(index);
    const FloatingPoint df_value = std::abs(sdf_value);
    TracyPlot("Distance", df_value);
    open_queue.pop();

    for (const Index3D& index_offset : kNeighborIndexOffsets) {
      // Compute the neighbor's distance if reached from the current voxel
      const Index3D neighbor_index = index + index_offset;
      FloatingPoint neighbor_df_candidate =
          minDistanceTo(neighbor_index, sdf_parent, min_cell_width);
      if (max_distance_ <= neighbor_df_candidate) {
        continue;
      }

      // Get the neighbor's SDF value
      auto& [neighbor_sdf_parent, neighbor_sdf_value] =
          sdf.getOrAllocateValue(neighbor_index);

      // If the neighbor is uninitialized, get its sign from the occupancy map
      const bool neighbor_uninitialized =
          neighbor_sdf_parent == sdf.getDefaultValue().parent;
      if (neighbor_uninitialized) {
        const FloatingPoint neighbor_occupancy =
            occupancy_query_accelerator.getCellValue(neighbor_index);
        // Never initialize or update unknown cells
        if (classifier_.is(neighbor_occupancy, Occupancy::kUnobserved)) {
          continue;
        }
        // Set the sign
        if (classifier_.is(neighbor_occupancy, Occupancy::kOccupied)) {
          neighbor_sdf_value = -sdf.getDefaultValue().distance;
        }
      }

      // Handle sign changes when propagating across the surface
      const bool crossed_surface =
          std::signbit(neighbor_sdf_value) != std::signbit(sdf_value);
      if (crossed_surface) {
        if (neighbor_sdf_value < 0.f) {
          DCHECK_LE(df_value, half_max_neighbor_distance_offset);
          neighbor_df_candidate =
              minDistanceTo(neighbor_index, index, min_cell_width);
        } else {
          continue;
        }
      }

      // Update the neighbor's SDF value
      const FloatingPoint neighbor_df = std::abs(neighbor_sdf_value);
      if (neighbor_df_candidate < neighbor_df) {
        neighbor_sdf_value =
            std::copysign(neighbor_df_candidate, neighbor_sdf_value);
        if (crossed_surface) {
          neighbor_sdf_parent = index;
        } else {
          neighbor_sdf_parent = sdf_parent;
        }
      }

      // If the neighbor is not yet in the open queue, add it
      if (neighbor_uninitialized) {
        open_queue.push(neighbor_df_candidate, neighbor_index);
      }
    }
  }
}

FloatingPoint FullEuclideanSDFGenerator::minDistanceTo(
    const Index3D& child, const Index3D& parent, FloatingPoint min_cell_width) {
  const Point3D child_center =
      convert::indexToCenterPoint(child, min_cell_width);

  const Point3D parent_min_corner =
      convert::indexToMinCorner(parent, min_cell_width);
  const Point3D parent_max_corner =
      parent_min_corner + Vector3D::Constant(min_cell_width);
  const Point3D closest_point =
      child_center.cwiseMax(parent_min_corner).cwiseMin(parent_max_corner);

  const FloatingPoint min_distance = (child_center - closest_point).norm();
  return min_distance;
}
}  // namespace wavemap
