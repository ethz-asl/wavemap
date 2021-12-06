#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_INL_H_

#include <stack>
#include <vector>

#include "wavemap_2d/datastructure/pointcloud.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename CellTypeT>
bool Quadtree<CellTypeT>::hasCell(const Index& index) const {
  const NodeIndex node_index = computeNodeIndexFromIndex(index, max_depth_);
  const Node<CellDataSpecialized>* node = getNode(node_index);
  return node;
}

template <typename CellTypeT>
FloatingPoint Quadtree<CellTypeT>::getCellValue(const Index& index) const {
  const NodeIndex node_index = computeNodeIndexFromIndex(index, max_depth_);
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
  const NodeIndex node_index = computeNodeIndexFromIndex(index, max_depth_);
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
  const NodeIndex node_index = computeNodeIndexFromIndex(index, max_depth_);
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
      parent_node.getChildPtr(index.computeRelativeChildIndex()) = nullptr;
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
    if (!current_parent->hasAllocatedChildren()) {
      if (auto_allocate) {
        current_parent->allocateChildren();
      } else {
        return nullptr;
      }
    }

    // Check if the child is allocated
    if (!current_parent->getChildPtr(child_index)) {
      if (auto_allocate) {
        current_parent->getChildPtr(child_index) =
            new Node<CellDataSpecialized>;
      } else {
        return nullptr;
      }
    }

    current_parent = current_parent->getChildPtr(child_index);
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
    if (!current_parent->hasAllocatedChildren()) {
      return nullptr;
    }

    // Check if the child is allocated
    if (!current_parent->getChildPtr(child_index)) {
      return nullptr;
    }

    current_parent = current_parent->getChildPtr(child_index);
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
