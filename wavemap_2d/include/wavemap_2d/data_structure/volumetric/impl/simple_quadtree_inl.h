#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_SIMPLE_QUADTREE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_SIMPLE_QUADTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap_2d {
template <typename CellT>
void SimpleQuadtree<CellT>::prune() {
  std::function<void(FloatingPoint, NodeType&)> recursive_fn =
      [&recursive_fn](FloatingPoint parent_value, NodeType& node) {
        // Process the children first
        const FloatingPoint node_value = parent_value + node.data();
        if (node.hasChildrenArray()) {
          for (QuadtreeIndex::RelativeChild child_idx = 0;
               child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
            if (node.hasChild(child_idx)) {
              recursive_fn(node_value, *node.getChild(child_idx));
            } else if (kEpsilon < std::abs(node_value)) {
              // Always propagate non-zero internal node value down to leaves
              recursive_fn(node_value, *node.template allocateChild(child_idx));
            }
          }
        }

        if (node.hasChildrenArray()) {
          // Check whether the node's children are all identical leaves
          bool all_children_are_identical_leaves = true;
          const FloatingPoint first_child_value =
              node.getChild(0) ? node.getChild(0)->data() : 0.f;
          for (QuadtreeIndex::RelativeChild child_idx = 0;
               child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
            // Check whether the child has children (i.e. is not a leaf)
            const NodeType* child = node.getChild(child_idx);
            if (child && child->hasChildrenArray()) {
              all_children_are_identical_leaves = false;
              break;
            }
            // Check whether this child's value differs from the first child
            const FloatingPoint child_value = child ? child->data() : 0.f;
            if (kEpsilon < std::abs(child_value - first_child_value)) {
              all_children_are_identical_leaves = false;
              break;
            }
          }

          if (all_children_are_identical_leaves) {
            // Prune the node's children if they're all identical leaves
            node.data() = first_child_value;
            node.deleteChildrenArray();
          } else {
            // After pruning, all internal node values are zero
            node.data() = 0.f;
          }
        } else {
          // Threshold the leaf values
          node.data() = CellT::threshold(node_value);
        }
      };

  recursive_fn(0.f, quadtree_.getRootNode());
}

template <typename CellT>
Index SimpleQuadtree<CellT>::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT>
Index SimpleQuadtree<CellT>::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT>
QuadtreeIndex::ChildArray SimpleQuadtree<CellT>::getFirstChildIndices() const {
  QuadtreeIndex::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential min candidates
template <typename CellT>
Index SimpleQuadtree<CellT>::getMinIndex() const {
  if (empty()) {
    return {};
  }

  std::stack<std::pair<QuadtreeIndex, const NodeType&>> stack;
  stack.template emplace(getInternalRootNodeIndex(), quadtree_.getRootNode());
  Index min_index = getMaxPossibleIndex();
  while (!stack.empty()) {
    const QuadtreeIndex internal_node_index = stack.top().first;
    const NodeType& node = stack.top().second;
    stack.pop();

    if (node.hasChildrenArray()) {
      for (QuadtreeIndex::RelativeChild child_idx = 0;
           child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
        if (node.hasChild(child_idx)) {
          const QuadtreeIndex child_node_index =
              internal_node_index.computeChildIndex(child_idx);
          const NodeType& child_node = *node.getChild(child_idx);
          stack.template emplace(child_node_index, child_node);
        }
      }
    } else if (OccupancyState::isObserved(node.data())) {
      const Index index =
          convert::nodeIndexToMinCornerIndex(internal_node_index);
      min_index = min_index.cwiseMin(index);
    }
  }

  return toExternalIndex(min_index);
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential max candidates
template <typename CellT>
Index SimpleQuadtree<CellT>::getMaxIndex() const {
  if (empty()) {
    return {};
  }

  std::stack<std::pair<QuadtreeIndex, const NodeType&>> stack;
  stack.template emplace(getInternalRootNodeIndex(), quadtree_.getRootNode());
  Index max_index = getMinPossibleIndex();
  while (!stack.empty()) {
    const QuadtreeIndex internal_node_index = stack.top().first;
    const NodeType& node = stack.top().second;
    stack.pop();

    if (node.hasChildrenArray()) {
      for (QuadtreeIndex::RelativeChild child_idx = 0;
           child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
        if (node.hasChild(child_idx)) {
          const QuadtreeIndex child_node_index =
              internal_node_index.computeChildIndex(child_idx);
          const NodeType& child_node = *node.getChild(child_idx);
          stack.template emplace(child_node_index, child_node);
        }
      }
    } else if (OccupancyState::isObserved(node.data())) {
      const Index index =
          convert::nodeIndexToMaxCornerIndex(internal_node_index);
      max_index = max_index.cwiseMax(index);
    }
  }

  return toExternalIndex(max_index);
}

template <typename CellT>
FloatingPoint SimpleQuadtree<CellT>::getCellValue(const Index& index) const {
  const NodeType* deepest_node_at_index = getDeepestNodeAtIndex(index);
  if (deepest_node_at_index) {
    return deepest_node_at_index->data();
  }
  return 0.f;
}

template <typename CellT>
void SimpleQuadtree<CellT>::setCellValue(const Index& index,
                                         FloatingPoint new_value) {
  const QuadtreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

template <typename CellT>
void SimpleQuadtree<CellT>::setCellValue(const QuadtreeIndex& node_index,
                                         FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  NodeType* node = quadtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

template <typename CellT>
void SimpleQuadtree<CellT>::addToCellValue(const Index& index,
                                           FloatingPoint update) {
  const QuadtreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

template <typename CellT>
void SimpleQuadtree<CellT>::addToCellValue(const QuadtreeIndex& node_index,
                                           FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  NodeType* node = quadtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() += update;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

template <typename CellT>
void SimpleQuadtree<CellT>::forEachLeaf(
    VolumetricDataStructure::IndexedLeafVisitorFunction visitor_fn) const {
  std::stack<std::pair<QuadtreeIndex, const NodeType&>> stack;
  stack.template emplace(getInternalRootNodeIndex(), quadtree_.getRootNode());
  while (!stack.empty()) {
    const QuadtreeIndex internal_node_index = stack.top().first;
    const NodeType& node = stack.top().second;
    stack.pop();

    if (node.hasChildrenArray()) {
      for (QuadtreeIndex::RelativeChild child_idx = 0;
           child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
        if (node.hasChild(child_idx)) {
          const QuadtreeIndex child_node_index =
              internal_node_index.computeChildIndex(child_idx);
          const NodeType& child_node = *node.getChild(child_idx);
          stack.template emplace(child_node_index, child_node);
        }
      }
    } else {
      const QuadtreeIndex node_index = toExternalNodeIndex(internal_node_index);
      visitor_fn(node_index, node.data());
    }
  }
}

template <typename CellT>
cv::Mat SimpleQuadtree<CellT>::getImage(bool /*use_color*/) const {
  // TODO(victorr): Implement this
  return {};
}

template <typename CellT>
bool SimpleQuadtree<CellT>::save(const std::string& /*file_path_prefix*/,
                                 bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool SimpleQuadtree<CellT>::load(const std::string& /*file_path_prefix*/,
                                 bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
const Node<typename CellT::Specialized>*
SimpleQuadtree<CellT>::getDeepestNodeAtIndex(const Index& index) const {
  const QuadtreeIndex deepest_possible_internal_node_index = toInternal(index);
  const NodeType* node = &quadtree_.getRootNode();
  const std::vector<QuadtreeIndex::RelativeChild> child_indices =
      deepest_possible_internal_node_index
          .computeRelativeChildIndices<kMaxHeight>();
  for (const QuadtreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return node;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_SIMPLE_QUADTREE_INL_H_
