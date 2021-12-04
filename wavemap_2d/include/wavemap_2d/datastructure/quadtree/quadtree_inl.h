#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_INL_H_

#include <stack>
#include <vector>

#include "wavemap_2d/pointcloud.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename NodeDataType>
NodeIndex Quadtree<NodeDataType>::computeNodeIndexFromCenter(
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

template <typename NodeDataType>
Point Quadtree<NodeDataType>::computeNodeCenterFromIndex(
    const NodeIndex& index) const {
  const Vector node_halved_diagonal = getNodeHalvedDiagonalAtDepth(index.depth);
  return computeNodeCornerFromIndex(index) + node_halved_diagonal;
}

template <typename NodeDataType>
Point Quadtree<NodeDataType>::computeNodeCornerFromIndex(
    const NodeIndex& index) const {
  const FloatingPoint width = getNodeWidthAtDepth(index.depth);
  const Vector root_halved_diagonal = getNodeHalvedDiagonalAtDepth(0u);
  return index.position.cast<FloatingPoint>() * width - root_halved_diagonal;
}

template <typename NodeDataType>
bool Quadtree<NodeDataType>::removeNodeWithIndex(const NodeIndex& index) {
  Node<NodeDataType> node_ptr = getNodeByIndex(index);
  if (node_ptr) {
    delete node_ptr;
    // Set the parent node's pointer to the child we just deleted to nullptr
    NodeIndex parent_index = index.computeParentIndex();
    Node<NodeDataType> parent_node =
        getNodeByIndex(parent_index, /*auto_allocate*/ false);
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

template <typename NodeDataType>
NodeDataType* Quadtree<NodeDataType>::getNodeDataByIndex(
    const NodeIndex& index, const bool auto_allocate) {
  Node<NodeDataType>* node = getNodeByIndex(index, auto_allocate);
  if (node) {
    return node->getNodeDataPtr();
  } else {
    return nullptr;
  }
}

template <typename NodeDataType>
Pointcloud Quadtree<NodeDataType>::getLeaveCenters(
    const NodeIndexElement max_depth) const {
  // TODO(victorr): Extend the OctreeIterator to take a max_depth param,
  //                then use that instead of reimplementing traversal here
  using IndexConstPointerPair =
      typename Node<NodeDataType>::IndexConstPointerPair;
  std::vector<Point> points;

  std::stack<IndexConstPointerPair> node_queue;
  IndexConstPointerPair root_node = {NodeIndex(), &root_node_};
  node_queue.emplace(root_node);

  while (!node_queue.empty()) {
    IndexConstPointerPair node = node_queue.top();
    node_queue.pop();

    if (node.ptr->hasAllocatedChildren() && node.index.depth <= max_depth) {
      for (NodeRelativeChildIndex relative_child_idx = 0;
           relative_child_idx < NodeIndex::kNumChildren; ++relative_child_idx) {
        if (node.ptr->hasChild(relative_child_idx)) {
          IndexConstPointerPair child_node = {
              node.index.computeChildIndex(relative_child_idx),
              node.ptr->getChildConstPtr(relative_child_idx)};
          node_queue.emplace(child_node);
        }
      }
    } else {
      Point center = computeNodeCenterFromIndex(node.index);
      points.push_back(center);
    }
  }

  return Pointcloud(points);
}

template <typename NodeDataType>
std::vector<PointWithValue> Quadtree<NodeDataType>::getLeaveValues(
    const NodeIndexElement max_depth) const {
  // TODO(victorr): Extend the OctreeIterator to take a max_depth param,
  //                then use that instead of reimplementing traversal here

  using IndexConstPointerPair =
      typename Node<NodeDataType>::IndexConstPointerPair;
  std::vector<PointWithValue> pointcloud;

  std::stack<IndexConstPointerPair> node_queue;
  IndexConstPointerPair root_node = {NodeIndex(), &root_node_};
  node_queue.emplace(root_node);

  while (!node_queue.empty()) {
    IndexConstPointerPair node = node_queue.top();
    node_queue.pop();

    if (node.ptr->hasAllocatedChildren() && node.index.depth <= max_depth) {
      for (NodeRelativeChildIndex relative_child_idx = 0;
           relative_child_idx < NodeIndex::kNumChildren; ++relative_child_idx) {
        if (node.ptr->hasChild(relative_child_idx)) {
          IndexConstPointerPair child_node = {
              node.index.computeChildIndex(relative_child_idx),
              node.ptr->getChildConstPtr(relative_child_idx)};
          node_queue.emplace(child_node);
        }
      }
    } else {
      PointWithValue point_with_value;
      point_with_value.position = computeNodeCenterFromIndex(node.index);
      point_with_value.value = node.ptr->getNodeDataConstPtr()->value;
      pointcloud.push_back(point_with_value);
    }
  }

  return pointcloud;
}

template <typename NodeDataType>
Node<NodeDataType>* Quadtree<NodeDataType>::getNodeByIndex(
    const NodeIndex& index, bool auto_allocate) {
  Node<NodeDataType>* current_parent = &root_node_;
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
        current_parent->getChildPtr(child_index) = new Node<NodeDataType>;
      } else {
        return nullptr;
      }
    }

    current_parent = current_parent->getChildPtr(child_index);
  }

  return current_parent;
}

template <typename NodeDataType>
FloatingPoint Quadtree<NodeDataType>::computeNodeWidthAtDepth(
    NodeIndexElement depth) {
  return root_node_width_ / std::exp2(depth);
}

template <typename NodeDataType>
Vector Quadtree<NodeDataType>::computeNodeHalvedDiagonalAtDepth(
    NodeIndexElement depth) {
  return Vector::Constant(0.5f) * computeNodeWidthAtDepth(depth);
}

template <typename NodeDataType>
void Quadtree<NodeDataType>::updateLookupTables() {
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
