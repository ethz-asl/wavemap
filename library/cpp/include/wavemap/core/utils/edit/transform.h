#ifndef WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_
#define WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_

#include <algorithm>
#include <memory>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/aabb.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/utils/edit/sum.h"
#include "wavemap/core/utils/iterate/grid_iterator.h"
#include "wavemap/core/utils/query/map_interpolator.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap::edit {
template <typename MapT>
std::unique_ptr<MapT> transform(
    const MapT& B_map, const Transformation3D& T_AB,
    const std::shared_ptr<ThreadPool>& thread_pool = nullptr) {
  using NodePtrType = typename MapT::Block::OctreeType::NodePtrType;
  const IndexElement tree_height = B_map.getTreeHeight();
  const FloatingPoint min_cell_width = B_map.getMinCellWidth();
  const FloatingPoint block_width =
      convert::heightToCellWidth(min_cell_width, tree_height);
  const FloatingPoint block_width_inv = 1.f / block_width;

  // Allocate blocks in the result map
  auto A_map = std::make_unique<MapT>(B_map.getConfig());
  B_map.forEachBlock([&A_map, &T_AB, tree_height, min_cell_width,
                      block_width_inv](const Index3D& block_index,
                                       const auto& /*block*/) {
    AABB<Point3D> A_aabb{};
    const auto B_block_aabb =
        convert::nodeIndexToAABB<3>({tree_height, block_index}, min_cell_width);
    for (int idx = 0; idx < B_block_aabb.kNumCorners; ++idx) {
      const Point3D& B_corner = B_block_aabb.corner_point(idx);
      const Point3D A_corner = T_AB * B_corner;
      A_aabb.includePoint(A_corner);
    }
    const Index3D A_block_index_min =
        convert::pointToFloorIndex(A_aabb.min, block_width_inv);
    const Index3D A_block_index_max =
        convert::pointToCeilIndex(A_aabb.max, block_width_inv);
    for (const auto& A_block_index :
         Grid(A_block_index_min, A_block_index_max)) {
      A_map->getOrAllocateBlock(A_block_index);
    }
  });

  // Populate map A by interpolating map B
  const Transformation3D T_BA = T_AB.inverse();
  QueryAccelerator B_accelerator{B_map};
  A_map->forEachBlock(
      [&B_accelerator, &T_BA, &thread_pool, tree_height, min_cell_width](
          const Index3D& block_index, auto& block) {
        // Indicate that the block has changed
        block.setLastUpdatedStamp();
        block.setNeedsPruning();

        // Get pointers to the root value and node, which contain the wavelet
        // scale and detail coefficients, respectively
        FloatingPoint* root_value_ptr = &block.getRootScale();
        NodePtrType root_node_ptr = &block.getRootNode();
        const OctreeIndex root_node_index{tree_height, block_index};

        // Recursively crop all nodes
        if (thread_pool) {
          thread_pool->add_task([B_accelerator, &T_BA, root_node_ptr,
                                 root_node_index, root_value_ptr,
                                 block_ptr = &block, min_cell_width]() mutable {
            detail::sumNodeRecursive<MapT>(
                *root_node_ptr, root_node_index, *root_value_ptr,
                [&B_accelerator, &T_BA](const Point3D& A_point) {
                  const auto B_point = T_BA * A_point;
                  return interpolate::trilinear(B_accelerator, B_point);
                },
                min_cell_width);
            block_ptr->prune();
          });
        } else {
          detail::sumNodeRecursive<MapT>(
              *root_node_ptr, root_node_index, *root_value_ptr,
              [&B_accelerator, &T_BA](const Point3D& A_point) {
                const auto B_point = T_BA * A_point;
                return interpolate::trilinear(B_accelerator, B_point);
              },
              min_cell_width);
          block.prune();
        }
      });

  // Wait for all parallel jobs to finish
  if (thread_pool) {
    thread_pool->wait_all();
  }

  return A_map;
}
}  // namespace wavemap::edit

#endif  // WAVEMAP_CORE_UTILS_EDIT_TRANSFORM_H_
