#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_INL_H_

#include <limits>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/datastructure/pointcloud.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename CellTypeT>
size_t Quadtree<CellTypeT>::size() const {
  size_t num_nodes = 0u;

  std::stack<const Node<CellDataSpecialized>*> stack;
  stack.template emplace(&root_node_);
  while (!stack.empty()) {
    ++num_nodes;
    const Node<CellDataSpecialized>* node = stack.top();
    stack.pop();

    if (node->hasAllocatedChildrenArray()) {
      for (NodeRelativeChildIndex child_idx = 0;
           child_idx < NodeIndex::kNumChildren; ++child_idx) {
        if (node->hasChild(child_idx)) {
          stack.template emplace(node->getChild(child_idx));
        }
      }
    }
  }

  // Subtract 1 to account for the fact that the root node is always allocated
  // and therefore isn't counted in the size
  --num_nodes;

  return num_nodes;
}

template <typename CellTypeT>
size_t Quadtree<CellTypeT>::getMemoryUsage() const {
  size_t memory_usage = 0u;

  std::stack<const Node<CellDataSpecialized>*> stack;
  stack.template emplace(&root_node_);
  while (!stack.empty()) {
    const Node<CellDataSpecialized>* node = stack.top();
    stack.pop();
    memory_usage += node->getMemoryUsage();

    if (node->hasAllocatedChildrenArray()) {
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

// TODO(victorr): Replace this with an implementation that only expands
//                potential min candidates
template <typename CellTypeT>
Index Quadtree<CellTypeT>::getMinIndex() const {
  Index min_index = Index::Constant(std::numeric_limits<IndexElement>::max());

  std::stack<std::pair<NodeIndex, const Node<CellDataSpecialized>*>> stack;
  stack.template emplace(NodeIndex{}, &root_node_);
  while (!stack.empty()) {
    const NodeIndex node_index = stack.top().first;
    const Node<CellDataSpecialized>* node = stack.top().second;
    stack.pop();

    if (node->hasAllocatedChildrenArray()) {
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
      const Index index =
          ((luts_.node_widths_at_depth_[node_index.depth] *
                node_index.position.template cast<FloatingPoint>() -
            luts_.node_halved_diagonals_at_depth_[0]) /
           resolution_)
              .array()
              .round()
              .template cast<IndexElement>();
      min_index = min_index.cwiseMin(index);
    }
  }

  return min_index;
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential max candidates
template <typename CellTypeT>
Index Quadtree<CellTypeT>::getMaxIndex() const {
  Index max_index =
      Index::Constant(std::numeric_limits<IndexElement>::lowest());

  std::stack<std::pair<NodeIndex, const Node<CellDataSpecialized>*>> stack;
  stack.template emplace(NodeIndex{}, &root_node_);
  while (!stack.empty()) {
    const NodeIndex node_index = stack.top().first;
    const Node<CellDataSpecialized>* node = stack.top().second;
    stack.pop();

    if (node->hasAllocatedChildrenArray()) {
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
      const Index index =
          ((luts_.node_widths_at_depth_[node_index.depth] *
                node_index.position.template cast<FloatingPoint>() -
            luts_.node_halved_diagonals_at_depth_[0]) /
           resolution_)
              .array()
              .round()
              .template cast<IndexElement>();
      max_index = max_index.cwiseMax(index);
    }
  }

  return max_index;
}

template <typename CellTypeT>
bool Quadtree<CellTypeT>::hasCell(const Index& index) const {
  const NodeIndex node_index =
      computeNodeIndexFromIndexAndDepth(index, max_depth_);
  const Node<CellDataSpecialized>* node = getNode(node_index);
  return node;
}

template <typename CellTypeT>
FloatingPoint Quadtree<CellTypeT>::getCellValue(const Index& index) const {
  const NodeIndex node_index =
      computeNodeIndexFromIndexAndDepth(index, max_depth_);
  const Node<CellDataSpecialized>* node = getNode(node_index);
  if (node) {
    return node->data();
  } else {
    return 0.f;
  }
}

template <typename CellTypeT>
void Quadtree<CellTypeT>::setCellValue(const Index& index,
                                       FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const NodeIndex node_index =
      computeNodeIndexFromIndexAndDepth(index, max_depth_);
  Node<CellDataSpecialized>* node = getNode(node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellTypeT>
void Quadtree<CellTypeT>::addToCellValue(const Index& index,
                                         FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const NodeIndex node_index =
      computeNodeIndexFromIndexAndDepth(index, max_depth_);
  Node<CellDataSpecialized>* node = getNode(node_index, kAutoAllocate);
  if (node) {
    node->data() = CellTypeT::add(node->data(), update);
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellTypeT>
cv::Mat Quadtree<CellTypeT>::getImage(bool /*use_color*/) const {
  // TODO(victorr): Implement this
  return {};
}

template <typename CellTypeT>
bool Quadtree<CellTypeT>::save(const std::string& /*file_path_prefix*/,
                               bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellTypeT>
bool Quadtree<CellTypeT>::load(const std::string& /*file_path_prefix*/,
                               bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellTypeT>
NodeIndex Quadtree<CellTypeT>::computeNodeIndexFromIndexAndDepth(
    const Index& index, NodeIndexElement depth) const {
  // TODO(victorr): Compute it in integer form, instead of round tripping
  //                through real coordinates
  return computeNodeIndexFromCenter(
      resolution_ * index.template cast<FloatingPoint>(), depth);
}

template <typename CellTypeT>
NodeIndex Quadtree<CellTypeT>::computeNodeIndexFromCenter(
    const Point& center, NodeIndexElement depth) const {
  NodeIndex index;
  const FloatingPoint width = getNodeWidthAtDepth(depth);
  const Vector root_halved_diagonal = getNodeHalvedDiagonalAtDepth(0u);
  const Vector node_halved_diagonal = getNodeHalvedDiagonalAtDepth(depth);

  index.position =
      ((center + root_halved_diagonal - node_halved_diagonal) / width)
          .cast<NodeIndexElement>();
  index.depth = depth;

  return index;
}

template <typename CellTypeT>
Point Quadtree<CellTypeT>::computeNodeCenterFromIndex(
    const NodeIndex& index) const {
  const Vector node_halved_diagonal = getNodeHalvedDiagonalAtDepth(index.depth);
  return computeNodeCornerFromIndex(index) + node_halved_diagonal;
}

template <typename CellTypeT>
Point Quadtree<CellTypeT>::computeNodeCornerFromIndex(
    const NodeIndex& index) const {
  const FloatingPoint width = getNodeWidthAtDepth(index.depth);
  const Vector root_halved_diagonal = getNodeHalvedDiagonalAtDepth(0u);
  return index.position.cast<FloatingPoint>() * width - root_halved_diagonal;
}

template <typename CellTypeT>
bool Quadtree<CellTypeT>::removeNode(const NodeIndex& index) {
  Node<CellDataSpecialized> node_ptr = getNode(index);
  if (node_ptr) {
    delete node_ptr;
    // Set the parent node's pointer to the child we just deleted to nullptr
    NodeIndex parent_index = index.computeParentIndex();
    Node<CellDataSpecialized> parent_node =
        getNode(parent_index, /*auto_allocate*/ false);
    if (parent_node) {
      parent_node.getChild(index.computeRelativeChildIndex()) = nullptr;
    } else {
      LOG(ERROR) << "Removed child node that was already orphaned. This should "
                    "never happen.";
    }
    return true;
  } else {
    return false;
  }
}

template <typename CellTypeT>
Node<typename CellTypeT::Specialized>* Quadtree<CellTypeT>::getNode(
    const NodeIndex& index, bool auto_allocate) {
  Node<CellDataSpecialized>* current_parent = &root_node_;
  std::vector<NodeRelativeChildIndex> child_indices =
      index.computeRelativeChildIndices();
  for (const NodeRelativeChildIndex child_index : child_indices) {
    // Check if the child pointer array is allocated
    if (!current_parent->hasAllocatedChildrenArray()) {
      if (auto_allocate) {
        current_parent->allocateChildrenArray();
      } else {
        return nullptr;
      }
    }

    // Check if the child is allocated
    if (!current_parent->getChild(child_index)) {
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

template <typename CellTypeT>
const Node<typename CellTypeT::Specialized>* Quadtree<CellTypeT>::getNode(
    const NodeIndex& index) const {
  const Node<CellDataSpecialized>* current_parent = &root_node_;
  std::vector<NodeRelativeChildIndex> child_indices =
      index.computeRelativeChildIndices();
  for (const NodeRelativeChildIndex child_index : child_indices) {
    // Check if the child pointer array is allocated
    if (!current_parent->hasAllocatedChildrenArray()) {
      return nullptr;
    }

    // Check if the child is allocated
    if (!current_parent->getChild(child_index)) {
      return nullptr;
    }

    current_parent = current_parent->getChild(child_index);
  }

  return current_parent;
}

template <typename CellTypeT>
FloatingPoint Quadtree<CellTypeT>::computeNodeWidthAtDepth(
    NodeIndexElement depth) {
  return root_node_width_ / std::exp2(depth);
}

template <typename CellTypeT>
Vector Quadtree<CellTypeT>::computeNodeHalvedDiagonalAtDepth(
    NodeIndexElement depth) {
  return Vector::Constant(0.5f) * computeNodeWidthAtDepth(depth);
}

template <typename CellTypeT>
void Quadtree<CellTypeT>::updateLookupTables() {
  // Update the cache sizes
  luts_.node_widths_at_depth_.resize(max_depth_ + 1);
  luts_.node_halved_diagonals_at_depth_.resize(max_depth_ + 1);

  // Update the cached values
  for (NodeIndexElement depth = 0; depth <= max_depth_; ++depth) {
    luts_.node_widths_at_depth_[depth] = computeNodeWidthAtDepth(depth);
    luts_.node_halved_diagonals_at_depth_[depth] =
        computeNodeHalvedDiagonalAtDepth(depth);
  }
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_INL_H_
