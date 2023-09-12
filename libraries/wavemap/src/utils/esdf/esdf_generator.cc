#include "wavemap/utils/esdf/esdf_generator.h"

#include <iostream>

#include <tracy/Tracy.hpp>

#include "wavemap/utils/esdf/bucket_queue.h"

namespace wavemap {
std::array<Index3D, 26> generateNeighborIndexOffsets() {
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

std::array<FloatingPoint, 26> generateNeighborDistanceOffsets(
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

HashedBlocks generateEsdf(const HashedWaveletOctree& occupancy_layer,
                          FloatingPoint occupancy_threshold,
                          FloatingPoint max_distance) {
  ZoneScoped;
  const FloatingPoint min_cell_width = occupancy_layer.getMinCellWidth();

  VolumetricDataStructureConfig config{min_cell_width, 0.f, max_distance};
  HashedBlocks esdf_layer(config, max_distance);

  const int num_bins =
      static_cast<int>(std::ceil(max_distance / min_cell_width));
  BucketQueue<Index3D> open{num_bins, max_distance};

  {
    ZoneScopedN("seedEsdf");
    LOG(INFO) << "Inserting occupied voxels into ESDF (Seeding)";
    occupancy_layer.forEachLeaf(
        [&esdf_layer, &open, min_cell_width, occupancy_threshold](
            const OctreeIndex& node_index, FloatingPoint occupancy) {
          if (occupancy_threshold < occupancy) {
            const Index3D min_corner =
                convert::nodeIndexToMinCornerIndex(node_index);
            const Index3D max_corner =
                convert::nodeIndexToMaxCornerIndex(node_index);
            const Index3D ones = Index3D::Ones();
            for (const Index3D& index :
                 Grid<3>(min_corner - ones, max_corner + ones)) {
              const Index3D nearest_inner_index =
                  index.cwiseMax(min_corner).cwiseMin(max_corner);
              const bool voxel_is_inside = (index == nearest_inner_index);
              if (voxel_is_inside) {
                esdf_layer.setCellValue(index, 0.f);
                open.push(index, 0.f);
              } else {
                const FloatingPoint distance_to_surface =
                    0.5f * min_cell_width *
                    (index - nearest_inner_index).cast<FloatingPoint>().norm();
                FloatingPoint& esdf = *esdf_layer.accessCellData(index, true);

                if (distance_to_surface < esdf) {
                  esdf = distance_to_surface;
                  open.push(index, distance_to_surface);
                }
              }
            }
          }
        });
  }

  const std::array neighbor_index_offsets = generateNeighborIndexOffsets();
  const std::array neighbor_distance_offsets =
      generateNeighborDistanceOffsets(min_cell_width);
  CHECK_EQ(neighbor_index_offsets.size(), neighbor_distance_offsets.size());

  {
    ZoneScopedN("propagateEsdf");
    LOG(INFO) << "Propagating ESDF into free space (Fast Marching)";
    while (!open.empty()) {
      const Index3D index = open.front();
      open.pop();

      const FloatingPoint esdf_value = esdf_layer.getCellValue(index);

      for (size_t neighbor_idx = 0;
           neighbor_idx < neighbor_index_offsets.size(); ++neighbor_idx) {
        const Index3D& neighbor_index =
            index + neighbor_index_offsets[neighbor_idx];

        const FloatingPoint neighbor_occupancy =
            occupancy_layer.getCellValue(neighbor_index);
        const bool neighbor_is_unobserved =
            std::abs(neighbor_occupancy) < 1e-4f;
        const bool neighbor_is_occupied =
            occupancy_threshold <= neighbor_occupancy;
        if (neighbor_is_unobserved || neighbor_is_occupied) {
          continue;
        }

        const FloatingPoint neighbor_distance =
            esdf_value + neighbor_distance_offsets[neighbor_idx];
        FloatingPoint& neighbor_esdf =
            *esdf_layer.accessCellData(neighbor_index, true);

        if (neighbor_distance < neighbor_esdf) {
          neighbor_esdf = neighbor_distance;
          open.push(neighbor_index, neighbor_distance);
        }
      }
    }
  }

  return esdf_layer;
}
}  // namespace wavemap
