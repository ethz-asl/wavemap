#ifndef WAVEMAP_CORE_UTILS_EDIT_IMPL_MULTIPLY_INL_H_
#define WAVEMAP_CORE_UTILS_EDIT_IMPL_MULTIPLY_INL_H_

#include <memory>

namespace wavemap::edit {
namespace detail {
template <typename MapT>
void multiplyNodeRecursive(typename MapT::Block::OctreeType::NodeRefType node,
                           FloatingPoint multiplier) {
  // Multiply
  node.data() *= multiplier;

  // Recursively handle all child nodes
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (auto child_node = node.getChild(child_idx); child_node) {
      multiplyNodeRecursive<MapT>(*child_node, multiplier);
    }
  }
}
}  // namespace detail

template <typename MapT>
void multiply(MapT& map, FloatingPoint multiplier,
              const std::shared_ptr<ThreadPool>& thread_pool) {
  using NodePtrType = typename MapT::Block::OctreeType::NodePtrType;

  // Process all blocks
  map.forEachBlock(
      [&thread_pool, multiplier](const Index3D& /*block_index*/, auto& block) {
        // Indicate that the block has changed
        block.setLastUpdatedStamp();

        // Multiply the block's average value (wavelet scale coefficient)
        FloatingPoint& root_value = block.getRootScale();
        root_value *= multiplier;

        // Recursively multiply all node values (wavelet detail coefficients)
        NodePtrType root_node_ptr = &block.getRootNode();
        if (thread_pool) {
          thread_pool->add_task([root_node_ptr, multiplier]() {
            detail::multiplyNodeRecursive<MapT>(*root_node_ptr, multiplier);
          });
        } else {
          detail::multiplyNodeRecursive<MapT>(*root_node_ptr, multiplier);
        }
      });

  // Wait for all parallel jobs to finish
  if (thread_pool) {
    thread_pool->wait_all();
  }
}
}  // namespace wavemap::edit

#endif  // WAVEMAP_CORE_UTILS_EDIT_IMPL_MULTIPLY_INL_H_
