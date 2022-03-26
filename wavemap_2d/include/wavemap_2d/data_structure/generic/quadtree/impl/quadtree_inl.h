#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_IMPL_QUADTREE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_IMPL_QUADTREE_INL_H_

#include <limits>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/data_structure/generic/pointcloud.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename CellT>
size_t Quadtree<CellT>::size() const {
  auto subtree_iterator = getIterator<TraversalOrder::kDepthFirstPreorder>();
  // NOTE: 1 is subtracted from the count to account for the fact that the root
  //       node is even allocated when the quadtree is empty.
  return std::distance(subtree_iterator.begin(), subtree_iterator.end()) - 1u;
}

template <typename CellT>
void Quadtree<CellT>::prune() {
  for (NodeType& node : getIterator<TraversalOrder::kDepthFirstPostorder>()) {
    if (node.hasChildrenArray()) {
      bool has_non_empty_child = false;
      for (int child_idx = 0; child_idx < NodeIndex::kNumChildren;
           ++child_idx) {
        NodeType* child_ptr = node.getChild(child_idx);
        if (child_ptr) {
          if (child_ptr->empty()) {
            node.deleteChild(child_idx);
          } else {
            has_non_empty_child = true;
          }
        }
      }

      // Free up the children array if it only contains null pointers
      if (!has_non_empty_child) {
        node.deleteChildrenArray();
      }
    }
  }
}

template <typename CellT>
Index Quadtree<CellT>::getMinPossibleIndex() const {
  return nodeIndexToIndex(NodeIndex{max_depth_, Index::Constant(0)});
}

template <typename CellT>
Index Quadtree<CellT>::getMaxPossibleIndex() const {
  return nodeIndexToIndex(NodeIndex{
      max_depth_, Index::Constant(constexpr_functions::exp2(max_depth_))});
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential min candidates
template <typename CellT>
Index Quadtree<CellT>::getMinIndex() const {
  Index min_index = Index::Constant(std::numeric_limits<IndexElement>::max());

  std::stack<std::pair<NodeIndex, const Node<CellDataSpecialized>*>> stack;
  stack.template emplace(NodeIndex{}, &root_node_);
  while (!stack.empty()) {
    const NodeIndex node_index = stack.top().first;
    const Node<CellDataSpecialized>* node = stack.top().second;
    stack.pop();

    if (node->hasChildrenArray()) {
      for (NodeRelativeChildIndex child_idx = 0;
           child_idx < NodeIndex::kNumChildren; ++child_idx) {
        if (node->hasChild(child_idx)) {
          const NodeIndex child_node_index =
              node_index.computeChildIndex(child_idx);
          const Node<CellDataSpecialized>* child_node =
              node->getChild(child_idx);
          stack.template emplace(child_node_index, child_node);
        }
      }
    } else {
      const Index index = nodeIndexToIndex(node_index);
      min_index = min_index.cwiseMin(index);
    }
  }

  return min_index;
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential max candidates
template <typename CellT>
Index Quadtree<CellT>::getMaxIndex() const {
  Index max_index =
      Index::Constant(std::numeric_limits<IndexElement>::lowest());

  std::stack<std::pair<NodeIndex, const Node<CellDataSpecialized>*>> stack;
  stack.template emplace(NodeIndex{}, &root_node_);
  while (!stack.empty()) {
    const NodeIndex node_index = stack.top().first;
    const Node<CellDataSpecialized>* node = stack.top().second;
    stack.pop();

    if (node->hasChildrenArray()) {
      for (NodeRelativeChildIndex child_idx = 0;
           child_idx < NodeIndex::kNumChildren; ++child_idx) {
        if (node->hasChild(child_idx)) {
          const NodeIndex child_node_index =
              node_index.computeChildIndex(child_idx);
          const Node<CellDataSpecialized>* child_node =
              node->getChild(child_idx);
          stack.template emplace(child_node_index, child_node);
        }
      }
    } else {
      const Index index = nodeIndexToIndex(node_index);
      max_index = max_index.cwiseMax(index);
    }
  }

  return max_index;
}

template <typename CellT>
bool Quadtree<CellT>::hasCell(const Index& index) const {
  const NodeIndex node_index = indexToNodeIndex(index);
  const Node<CellDataSpecialized>* node = getNode(node_index);
  return node;
}

template <typename CellT>
FloatingPoint Quadtree<CellT>::getCellValue(const Index& index) const {
  const NodeIndex node_index = indexToNodeIndex(index);
  const Node<CellDataSpecialized>* node = getNode(node_index);
  if (node) {
    return node->data();
  } else {
    return 0.f;
  }
}

template <typename CellT>
void Quadtree<CellT>::setCellValue(const Index& index,
                                   FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const NodeIndex node_index = indexToNodeIndex(index);
  Node<CellDataSpecialized>* node = getNode(node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
void Quadtree<CellT>::addToCellValue(const Index& index, FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const NodeIndex node_index = indexToNodeIndex(index);
  Node<CellDataSpecialized>* node = getNode(node_index, kAutoAllocate);
  if (node) {
    node->data() = CellT::add(node->data(), update);
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
size_t Quadtree<CellT>::getMemoryUsage() const {
  size_t memory_usage = 0u;

  std::stack<const Node<CellDataSpecialized>*> stack;
  stack.template emplace(&root_node_);
  while (!stack.empty()) {
    const Node<CellDataSpecialized>* node = stack.top();
    stack.pop();
    memory_usage += node->getMemoryUsage();

    if (node->hasChildrenArray()) {
      for (NodeRelativeChildIndex child_idx = 0;
           child_idx < NodeIndex::kNumChildren; ++child_idx) {
        if (node->hasChild(child_idx)) {
          stack.template emplace(node->getChild(child_idx));
        }
      }
    }
  }

  return memory_usage;
}

template <typename CellT>
cv::Mat Quadtree<CellT>::getImage(bool /*use_color*/) const {
  // TODO(victorr): Implement this
  return {};
}

template <typename CellT>
bool Quadtree<CellT>::save(const std::string& /*file_path_prefix*/,
                           bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool Quadtree<CellT>::load(const std::string& /*file_path_prefix*/,
                           bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool Quadtree<CellT>::removeNode(const NodeIndex& index) {
  NodeIndex parent_index = index.computeParentIndex();
  Node<CellDataSpecialized>* parent_node =
      getNode(parent_index, /*auto_allocate*/ false);
  if (parent_node) {
    return parent_node->deleteChild(index.computeRelativeChildIndex());
  }
  return false;
}

template <typename CellT>
Node<typename CellT::Specialized>* Quadtree<CellT>::getNode(
    const NodeIndex& index, bool auto_allocate) {
  Node<CellDataSpecialized>* current_parent = &root_node_;
  const std::vector<NodeRelativeChildIndex> child_indices =
      index.computeRelativeChildIndices();
  for (const NodeRelativeChildIndex child_index : child_indices) {
    // Check if the child is allocated
    if (!current_parent->hasChild(child_index)) {
      if (auto_allocate) {
        current_parent->allocateChild(child_index);
      } else {
        return nullptr;
      }
    }

    current_parent = current_parent->getChild(child_index);
  }

  return current_parent;
}

template <typename CellT>
const Node<typename CellT::Specialized>* Quadtree<CellT>::getNode(
    const NodeIndex& index) const {
  const Node<CellDataSpecialized>* current_parent = &root_node_;
  const std::vector<NodeRelativeChildIndex> child_indices =
      index.computeRelativeChildIndices();
  for (const NodeRelativeChildIndex child_index : child_indices) {
    // Check if the child is allocated
    if (!current_parent->hasChild(child_index)) {
      return nullptr;
    }

    current_parent = current_parent->getChild(child_index);
  }

  return current_parent;
}

template <typename CellT>
FloatingPoint Quadtree<CellT>::computeNodeWidthAtDepth(NodeIndexElement depth) {
  return root_node_width_ / std::exp2(depth);
}

template <typename CellT>
Vector Quadtree<CellT>::computeNodeHalvedDiagonalAtDepth(
    NodeIndexElement depth) {
  return Vector::Constant(0.5f) * computeNodeWidthAtDepth(depth);
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_IMPL_QUADTREE_INL_H_
