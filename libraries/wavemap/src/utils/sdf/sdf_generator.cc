#include "wavemap/utils/sdf/sdf_generator.h"

#include <tracy/Tracy.hpp>

namespace wavemap {
std::array<Index3D, 26> neighborhood::generateNeighborIndexOffsets() {
  std::array<Index3D, 26> neighbor_offsets{};
  size_t array_idx = 0u;
  for (const Index3D& index : Grid<3>(-Index3D::Ones(), Index3D::Ones())) {
    if (index != Index3D::Zero()) {
      neighbor_offsets[array_idx] = index;
      ++array_idx;
    }
  }
  return neighbor_offsets;
}

std::array<FloatingPoint, 26> neighborhood::generateNeighborDistanceOffsets(
    FloatingPoint cell_width) {
  std::array<FloatingPoint, 26> distance_offsets{};
  size_t array_idx = 0u;
  for (const Index3D& index_offset : generateNeighborIndexOffsets()) {
    distance_offsets[array_idx] =
        cell_width * index_offset.cast<FloatingPoint>().norm();
    ++array_idx;
  }
  return distance_offsets;
}

HashedBlocks QuasiEuclideanSDFGenerator::generate(
    const HashedWaveletOctree& occupancy_map) {
  ZoneScoped;
  // Initialize the SDF data structure
  const FloatingPoint min_cell_width = occupancy_map.getMinCellWidth();
  const VolumetricDataStructureConfig config{min_cell_width, 0.f,
                                             max_distance_};
  HashedBlocks sdf(config, max_distance_);

  // Initialize the bucketed priority queue
  const int num_bins =
      static_cast<int>(std::ceil(max_distance_ / min_cell_width));
  BucketQueue<Index3D> open{num_bins, max_distance_};

  // Seed and propagate the SDF
  seed(occupancy_map, sdf, open);
  propagate(occupancy_map, sdf, open);

  return sdf;
}

void QuasiEuclideanSDFGenerator::seed(const HashedWaveletOctree& occupancy_map,
                                      HashedBlocks& sdf,
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
    if (isUnobserved(node_occupancy) || isFree(node_occupancy)) {
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
      if (isUnobserved(occupancy) || isOccupied(occupancy)) {
        continue;
      }

      // Get the voxel's SDF value
      FloatingPoint& sdf_value = *sdf.accessCellData(index, true);
      const bool sdf_uninitialized =
          sdf.getDefaultCellValue() - kTolerance < sdf_value;

      // Update the voxel's SDF value
      const FloatingPoint distance_to_surface =
          0.5f * min_cell_width *
          (index - nearest_inner_index).cast<FloatingPoint>().norm();
      sdf_value = std::min(sdf_value, distance_to_surface);

      // If the voxel is not yet in the open queue, add it
      if (sdf_uninitialized) {
        open_queue.push(distance_to_surface, index);
      }
    }
  });
}

void QuasiEuclideanSDFGenerator::propagate(
    const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
    BucketQueue<Index3D>& open_queue) const {
  ZoneScoped;
  // Create an occupancy query accelerator
  QueryAccelerator occupancy_query_accelerator{occupancy_map};

  // Precompute the neighbor distance offsets
  const FloatingPoint min_cell_width = occupancy_map.getMinCellWidth();
  const auto neighbor_distance_offsets =
      neighborhood::generateNeighborDistanceOffsets(min_cell_width);
  CHECK_EQ(kNeighborIndexOffsets.size(), neighbor_distance_offsets.size());
  const FloatingPoint half_max_neighbor_distance_offset =
      std::sqrt(3.f) * min_cell_width / 2;

  // Propagate the distance
  while (!open_queue.empty()) {
    TracyPlot("QueueLength", static_cast<int64_t>(open_queue.size()));
    const Index3D index = open_queue.front();
    const FloatingPoint sdf_value = sdf.getCellValue(index);
    const FloatingPoint df_value = std::abs(sdf_value);
    TracyPlot("Distance", df_value);
    open_queue.pop();

    for (size_t neighbor_idx = 0; neighbor_idx < kNeighborIndexOffsets.size();
         ++neighbor_idx) {
      // Compute the neighbor's distance if reached from the current voxel
      FloatingPoint neighbor_df_candidate =
          df_value + neighbor_distance_offsets[neighbor_idx];
      if (max_distance_ <= neighbor_df_candidate) {
        continue;
      }

      // Get the neighbor's SDF value
      const Index3D& neighbor_index =
          index + kNeighborIndexOffsets[neighbor_idx];
      FloatingPoint& neighbor_sdf = *sdf.accessCellData(neighbor_index, true);

      // If the neighbor is uninitialized, get its sign from the occupancy map
      const bool neighbor_sdf_uninitialized =
          sdf.getDefaultCellValue() - kTolerance < neighbor_sdf;
      if (neighbor_sdf_uninitialized) {
        const FloatingPoint neighbor_occupancy =
            occupancy_query_accelerator.getCellValue(neighbor_index);
        // Never initialize or update unknown cells
        if (!isObserved(neighbor_occupancy)) {
          continue;
        }
        // Set the sign
        if (isOccupied(neighbor_occupancy)) {
          neighbor_sdf = -sdf.getDefaultCellValue();
        }
      }

      // Handle sign changes when propagating across the surface
      if (0.f < sdf_value && neighbor_sdf < 0.f) {
        // NOTE: When the opened cell and the neighbor cell have the same sign,
        //       the distance field value and offset are summed to obtain the
        //       unsigned neighbor distance. Whereas when moving across the
        //       surface, the df_value and offset have opposite signs and reduce
        //       each other instead.
        DCHECK_LE(df_value, half_max_neighbor_distance_offset);
        neighbor_df_candidate =
            neighbor_distance_offsets[neighbor_idx] - df_value;
      }

      // Update the neighbor's SDF value
      FloatingPoint neighbor_df = std::abs(neighbor_sdf);
      neighbor_df = std::min(neighbor_df, neighbor_df_candidate);
      neighbor_sdf = std::copysign(neighbor_df, neighbor_sdf);

      // If the neighbor is not yet in the open queue, add it
      if (neighbor_sdf_uninitialized) {
        open_queue.push(neighbor_df_candidate, neighbor_index);
      }
    }
  }
}
}  // namespace wavemap
