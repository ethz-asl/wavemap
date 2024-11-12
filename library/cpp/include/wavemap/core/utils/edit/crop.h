#ifndef WAVEMAP_CORE_UTILS_EDIT_CROP_H_
#define WAVEMAP_CORE_UTILS_EDIT_CROP_H_

#include "wavemap/core/common.h"

namespace wavemap {
template <typename MapType>
void cropLeavesBatch(typename MapType::Block::OctreeType::NodeRefType node,
                     const OctreeIndex& node_index, FloatingPoint& node_value,
                     const Point3D& t_W_center, FloatingPoint radius,
                     FloatingPoint min_cell_width) {
  // Decompress child values
  using Transform = typename MapType::Block::Transform;
  auto& node_details = node.data();
  auto child_values = Transform::backward({node_value, {node_details}});

  // Set all children whose center is outside the cropping sphere to zero
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    const OctreeIndex child_index = node_index.computeChildIndex(child_idx);
    const Point3D t_W_child =
        convert::nodeIndexToCenterPoint(child_index, min_cell_width);
    const FloatingPoint d_center_child = (t_W_child - t_W_center).norm();
    if (radius < d_center_child) {
      child_values[child_idx] = 0;
      if (0 < child_index.height) {
        node.eraseChild(child_idx);
      }
    }
  }

  // Compress
  const auto [new_value, new_details] =
      HashedChunkedWaveletOctreeBlock::Transform::forward(child_values);
  node_details = new_details;
  node_value = new_value;
}

template <typename MapType>
void cropNodeRecursive(typename MapType::Block::OctreeType::NodeRefType node,
                       const OctreeIndex& node_index, FloatingPoint& node_value,
                       const Point3D& t_W_center, FloatingPoint radius,
                       FloatingPoint min_cell_width,
                       IndexElement termination_height) {
  using NodeRefType = decltype(node);

  // Decompress child values
  using Transform = typename MapType::Block::Transform;
  auto& node_details = node.data();
  auto child_values = Transform::backward({node_value, {node_details}});

  // Handle each child
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    // If the node is fully inside the cropping sphere, do nothing
    const auto child_aabb =
        convert::nodeIndexToAABB(node_index, min_cell_width);
    if (child_aabb.maxDistanceTo(t_W_center) < radius) {
      continue;
    }

    // If the node is fully outside the cropping sphere, set it to zero
    auto& child_value = child_values[child_idx];
    if (radius < child_aabb.minDistanceTo(t_W_center)) {
      child_value = 0;
      node.eraseChild(child_idx);
      continue;
    }

    // Otherwise, continue at a higher resolution
    NodeRefType child_node = node.getOrAllocateChild(child_idx);
    const OctreeIndex child_index = node_index.computeChildIndex(child_idx);
    if (child_index.height <= termination_height + 1) {
      cropLeavesBatch<MapType>(child_node, child_index, child_value, t_W_center,
                               radius, min_cell_width);
    } else {
      cropNodeRecursive<MapType>(child_node, child_index, child_value,
                                 t_W_center, radius, min_cell_width,
                                 termination_height);
    }
  }

  // Compress
  const auto [new_value, new_details] = Transform::forward(child_values);
  node_details = new_details;
  node_value = new_value;
}

template <typename MapType>
void crop_to_sphere(const Point3D& t_W_center, FloatingPoint radius,
                    MapType& map, IndexElement termination_height) {
  const IndexElement tree_height = map.getTreeHeight();
  const FloatingPoint min_cell_width = map.getMinCellWidth();

  for (auto it = map.getHashMap().begin(); it != map.getHashMap().end();) {
    // Start by testing at the block level
    const Index3D& block_index = it->first;
    const auto block_node_index = OctreeIndex{tree_height, block_index};
    const auto block_aabb =
        convert::nodeIndexToAABB(block_node_index, min_cell_width);
    // If the block is fully inside the cropping sphere, do nothing
    if (block_aabb.maxDistanceTo(t_W_center) < radius) {
      ++it;
      continue;
    }
    // If the block is fully outside the cropping sphere, erase it entirely
    if (radius < block_aabb.minDistanceTo(t_W_center)) {
      it = map.getHashMap().erase(it);
      continue;
    }

    // Since the block overlaps with the sphere's boundary, we need to process
    // it at a higher resolution by recursing over its cells
    auto& block = it->second;
    cropNodeRecursive<MapType>(block.getRootNode(), block_node_index,
                               block.getRootScale(), t_W_center, radius,
                               min_cell_width, termination_height);
    block.setLastUpdatedStamp();

    ++it;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_EDIT_CROP_H_
