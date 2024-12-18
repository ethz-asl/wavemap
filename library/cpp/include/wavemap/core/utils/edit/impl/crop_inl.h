#ifndef WAVEMAP_CORE_UTILS_EDIT_IMPL_CROP_INL_H_
#define WAVEMAP_CORE_UTILS_EDIT_IMPL_CROP_INL_H_

#include <memory>

#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/utils/shape/intersection_tests.h"

namespace wavemap::edit {
namespace detail {
template <typename MapT, typename ShapeT>
void cropLeavesBatch(typename MapT::Block::OctreeType::NodeRefType node,
                     const OctreeIndex& node_index, FloatingPoint& node_value,
                     ShapeT&& mask, FloatingPoint min_cell_width) {
  // Decompress child values
  using Transform = typename MapT::Block::Transform;
  auto& node_details = node.data();
  auto child_values = Transform::backward({node_value, {node_details}});

  // Set all children whose center is outside the cropping shape to zero
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    const OctreeIndex child_index = node_index.computeChildIndex(child_idx);
    const Point3D t_W_child =
        convert::nodeIndexToCenterPoint(child_index, min_cell_width);
    if (!shape::is_inside(t_W_child, mask)) {
      child_values[child_idx] = 0.f;
      if (0 < child_index.height) {
        node.eraseChild(child_idx);
      }
    }
  }

  // Compress
  const auto [new_value, new_details] =
      MapT::Block::Transform::forward(child_values);
  node_details = new_details;
  node_value = new_value;
}

template <typename MapT, typename ShapeT>
void cropNodeRecursive(typename MapT::Block::OctreeType::NodeRefType node,
                       const OctreeIndex& node_index, FloatingPoint& node_value,
                       ShapeT&& mask, FloatingPoint min_cell_width,
                       IndexElement termination_height) {
  using NodeRefType = decltype(node);

  // Decompress child values
  using Transform = typename MapT::Block::Transform;
  auto& node_details = node.data();
  auto child_values = Transform::backward({node_value, {node_details}});

  // Handle each child
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    // If the node is fully inside the cropping shape, do nothing
    const OctreeIndex child_index = node_index.computeChildIndex(child_idx);
    const AABB<Point3D> child_aabb =
        convert::nodeIndexToAABB(child_index, min_cell_width);
    if (shape::is_inside(child_aabb, mask)) {
      continue;
    }

    // If the node is fully outside the cropping shape, set it to zero
    auto& child_value = child_values[child_idx];
    if (!shape::overlaps(child_aabb, mask)) {
      child_value = 0.f;
      node.eraseChild(child_idx);
      continue;
    }

    // Otherwise, continue at a higher resolution
    NodeRefType child_node = node.getOrAllocateChild(child_idx);
    if (child_index.height <= termination_height + 1) {
      cropLeavesBatch<MapT>(child_node, child_index, child_value, mask,
                            min_cell_width);
    } else {
      cropNodeRecursive<MapT>(child_node, child_index, child_value, mask,
                              min_cell_width, termination_height);
    }
  }

  // Compress
  const auto [new_value, new_details] = Transform::forward(child_values);
  node_details = new_details;
  node_value = new_value;
}
}  // namespace detail

template <typename MapT, typename ShapeT>
void crop(MapT& map, ShapeT mask, IndexElement termination_height,
          const std::shared_ptr<ThreadPool>& thread_pool) {
  using NodePtrType = typename MapT::Block::OctreeType::NodePtrType;
  const IndexElement tree_height = map.getTreeHeight();
  const FloatingPoint min_cell_width = map.getMinCellWidth();

  // Check all blocks
  for (auto it = map.getHashMap().begin(); it != map.getHashMap().end();) {
    // Start by testing at the block level
    const Index3D& block_index = it->first;
    const OctreeIndex block_node_index{tree_height, block_index};
    const auto block_aabb =
        convert::nodeIndexToAABB(block_node_index, min_cell_width);
    // If the block is fully inside the cropping shape, do nothing
    if (shape::is_inside(block_aabb, mask)) {
      ++it;
      continue;
    }
    // If the block is fully outside the cropping shape, erase it entirely
    if (!shape::overlaps(block_aabb, mask)) {
      it = map.getHashMap().erase(it);
      continue;
    }

    // Since the block overlaps with the shape's boundary, we need to process
    // it at a higher resolution by recursing over its cells
    auto& block = it->second;
    // Indicate that the block has changed
    block.setLastUpdatedStamp();
    // Get pointers to the root value and node, which contain the wavelet
    // scale and detail coefficients, respectively
    FloatingPoint* root_value_ptr = &block.getRootScale();
    NodePtrType root_node_ptr = &block.getRootNode();
    // Recursively crop all nodes
    if (thread_pool) {
      thread_pool->add_task([&mask, root_node_ptr, block_node_index,
                             root_value_ptr, min_cell_width,
                             termination_height]() {
        detail::cropNodeRecursive<MapT>(*root_node_ptr, block_node_index,
                                        *root_value_ptr, mask, min_cell_width,
                                        termination_height);
      });
    } else {
      detail::cropNodeRecursive<MapT>(*root_node_ptr, block_node_index,
                                      *root_value_ptr, mask, min_cell_width,
                                      termination_height);
    }
    // Advance to the next block
    ++it;
  }
  // Wait for all parallel jobs to finish
  if (thread_pool) {
    thread_pool->wait_all();
  }
}
}  // namespace wavemap::edit

#endif  // WAVEMAP_CORE_UTILS_EDIT_IMPL_CROP_INL_H_
