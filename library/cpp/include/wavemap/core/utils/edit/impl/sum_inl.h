#ifndef WAVEMAP_CORE_UTILS_EDIT_IMPL_SUM_INL_H_
#define WAVEMAP_CORE_UTILS_EDIT_IMPL_SUM_INL_H_

#include <memory>
#include <utility>

#include "wavemap/core/indexing/index_conversions.h"

namespace wavemap::edit {
namespace detail {
template <typename MapT, typename SamplingFn>
void sumLeavesBatch(typename MapT::Block::OctreeType::NodeRefType node,
                    const OctreeIndex& node_index, FloatingPoint& node_value,
                    SamplingFn&& sampling_function,
                    FloatingPoint min_cell_width) {
  // Decompress child values
  using Transform = typename MapT::Block::Transform;
  auto& node_details = node.data();
  auto child_values = Transform::backward({node_value, {node_details}});

  // Sum all children
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    const OctreeIndex child_index = node_index.computeChildIndex(child_idx);
    const Point3D t_W_child =
        convert::nodeIndexToCenterPoint(child_index, min_cell_width);
    child_values[child_idx] += sampling_function(t_W_child);
  }

  // Compress
  const auto [new_value, new_details] =
      MapT::Block::Transform::forward(child_values);
  node_details = new_details;
  node_value = new_value;
}

template <typename MapT, typename SamplingFn>
void sumNodeRecursive(typename MapT::Block::OctreeType::NodeRefType node,
                      const OctreeIndex& node_index, FloatingPoint& node_value,
                      SamplingFn&& sampling_function,
                      FloatingPoint min_cell_width,
                      IndexElement termination_height) {
  using NodeRefType = decltype(node);

  // Decompress child values
  using Transform = typename MapT::Block::Transform;
  auto& node_details = node.data();
  auto child_values = Transform::backward({node_value, {node_details}});

  // Handle each child
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    NodeRefType child_node = node.getOrAllocateChild(child_idx);
    const OctreeIndex child_index = node_index.computeChildIndex(child_idx);
    auto& child_value = child_values[child_idx];
    if (child_index.height <= termination_height + 1) {
      sumLeavesBatch<MapT>(child_node, child_index, child_value,
                           std::forward<SamplingFn>(sampling_function),
                           min_cell_width);
    } else {
      sumNodeRecursive<MapT>(child_node, child_index, child_value,
                             std::forward<SamplingFn>(sampling_function),
                             min_cell_width, termination_height);
    }
  }

  // Compress
  const auto [new_value, new_details] = Transform::forward(child_values);
  node_details = new_details;
  node_value = new_value;
}

template <typename MapT>
void sumNodeRecursive(
    typename MapT::Block::OctreeType::NodeRefType node_A,
    typename MapT::Block::OctreeType::NodeConstRefType node_B) {
  using NodeRefType = decltype(node_A);
  using NodeConstPtrType = typename MapT::Block::OctreeType::NodeConstPtrType;

  // Sum
  node_A.data() += node_B.data();

  // Recursively handle all child nodes
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    NodeConstPtrType child_node_B = node_B.getChild(child_idx);
    if (!child_node_B) {
      continue;
    }
    NodeRefType child_node_A = node_A.getOrAllocateChild(child_idx);
    sumNodeRecursive<MapT>(child_node_A, *child_node_B);
  }
}
}  // namespace detail

template <typename MapT, typename SamplingFn>
void sum(MapT& map, SamplingFn sampling_function,
         IndexElement termination_height,
         const std::shared_ptr<ThreadPool>& thread_pool) {
  using NodePtrType = typename MapT::Block::OctreeType::NodePtrType;
  const IndexElement tree_height = map.getTreeHeight();
  const FloatingPoint min_cell_width = map.getMinCellWidth();
  // Evaluate the sampling function over the entire map
  map.forEachBlock(
      [&thread_pool, &sampling_function, tree_height, min_cell_width,
       termination_height](const Index3D& block_index, auto& block) {
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
          thread_pool->add_task([root_node_ptr, root_node_index, root_value_ptr,
                                 block_ptr = &block, sampling_function,
                                 min_cell_width, termination_height]() {
            detail::sumNodeRecursive<MapT>(*root_node_ptr, root_node_index,
                                           *root_value_ptr, sampling_function,
                                           min_cell_width, termination_height);
            block_ptr->prune();
          });
        } else {
          detail::sumNodeRecursive<MapT>(*root_node_ptr, root_node_index,
                                         *root_value_ptr, sampling_function,
                                         min_cell_width, termination_height);
          block.prune();
        }
      });

  // Wait for all parallel jobs to finish
  if (thread_pool) {
    thread_pool->wait_all();
  }
}

template <typename MapT>
void sum(MapT& map_A, const MapT& map_B,
         const std::shared_ptr<ThreadPool>& thread_pool) {
  CHECK_EQ(map_A.getTreeHeight(), map_B.getTreeHeight());
  CHECK_EQ(map_A.getMinCellWidth(), map_B.getMinCellWidth());
  using NodePtrType = typename MapT::Block::OctreeType::NodePtrType;
  using NodeConstPtrType = typename MapT::Block::OctreeType::NodeConstPtrType;

  // Process all blocks
  map_B.forEachBlock(
      [&map_A, &thread_pool](const Index3D& block_index, const auto& block_B) {
        auto& block_A = map_A.getOrAllocateBlock(block_index);

        // Indicate that the block has changed
        block_A.setLastUpdatedStamp();
        block_A.setNeedsPruning();

        // Sum the blocks' average values (wavelet scale coefficient)
        block_A.getRootScale() += block_B.getRootScale();

        // Recursively sum all node values (wavelet detail coefficients)
        NodePtrType root_node_ptr_A = &block_A.getRootNode();
        NodeConstPtrType root_node_ptr_B = &block_B.getRootNode();
        if (thread_pool) {
          thread_pool->add_task([root_node_ptr_A, root_node_ptr_B,
                                 block_ptr_A = &block_A]() {
            detail::sumNodeRecursive<MapT>(*root_node_ptr_A, *root_node_ptr_B);
            block_ptr_A->prune();
          });
        } else {
          detail::sumNodeRecursive<MapT>(*root_node_ptr_A, *root_node_ptr_B);
          block_A.prune();
        }
      });

  // Wait for all parallel jobs to finish
  if (thread_pool) {
    thread_pool->wait_all();
  }
}
}  // namespace wavemap::edit

#endif  // WAVEMAP_CORE_UTILS_EDIT_IMPL_SUM_INL_H_
