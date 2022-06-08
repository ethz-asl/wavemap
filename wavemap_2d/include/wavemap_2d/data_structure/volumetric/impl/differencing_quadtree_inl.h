#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_DIFFERENCING_QUADTREE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_DIFFERENCING_QUADTREE_INL_H_

#include <limits>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap_2d {
template <typename CellT>
void DifferencingQuadtree<CellT>::prune() {
  std::function<void(FloatingPoint, Node<CellDataSpecialized>&)> recursive_fn =
      [&recursive_fn](FloatingPoint parent_value,
                      Node<CellDataSpecialized>& node) {
        // Process the children first
        const FloatingPoint node_value = parent_value + node.data();
        if (node.hasChildrenArray()) {
          for (QuadtreeIndex::RelativeChild child_idx = 0;
               child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
            if (node.hasChild(child_idx)) {
              recursive_fn(node_value, *node.getChild(child_idx));
            }
          }
        }

        if (node.hasChildrenArray()) {
          // Compute the average of all children
          FloatingPoint child_average = 0.f;
          for (int child_idx = 0; child_idx < QuadtreeIndex::kNumChildren;
               ++child_idx) {
            Node<CellDataSpecialized>* child = node.getChild(child_idx);
            if (child) {
              child_average += child->data();
            }
          }
          child_average /=
              static_cast<CellDataSpecialized>(QuadtreeIndex::kNumChildren);
          const bool child_average_is_non_zero =
              kEpsilon < std::abs(child_average);

          // Propagate the averages from the children to their parents
          if (child_average_is_non_zero) {
            node.data() += child_average;
          }

          // Subtract the average from the children and prune away zero leaves
          bool has_non_empty_child = false;
          for (int child_idx = 0; child_idx < QuadtreeIndex::kNumChildren;
               ++child_idx) {
            Node<CellDataSpecialized>* child = node.getChild(child_idx);
            if (child_average_is_non_zero) {
              if (child) {
                child->data() -= child_average;
              } else {
                child = node.template allocateChild(child_idx, -child_average);
              }
            }
            if (child) {
              if (std::abs(child->data()) < kEpsilon &&
                  !child->hasChildrenArray()) {
                node.deleteChild(child_idx);
              } else {
                has_non_empty_child = true;
              }
            }
          }

          // Free up the children array if it only contains null ptrs
          if (!has_non_empty_child) {
            node.deleteChildrenArray();
          }
        } else {
          // Threshold the leaf values
          node.data() = CellT::threshold(node_value) - parent_value;
        }
      };

  recursive_fn(0.f, quadtree_.getRootNode());
}

template <typename CellT>
Index DifferencingQuadtree<CellT>::getMinPossibleIndex() const {
  return toExternalIndex(QuadtreeIndex{0u, Index::Zero()});
}

template <typename CellT>
Index DifferencingQuadtree<CellT>::getMaxPossibleIndex() const {
  return toExternalIndex(
      QuadtreeIndex{0u, Index::Constant(int_math::exp2(kMaxHeight) - 1)});
}

template <typename CellT>
QuadtreeIndex::ChildArray DifferencingQuadtree<CellT>::getFirstChildIndices()
    const {
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
Index DifferencingQuadtree<CellT>::getMinIndex() const {
  Index min_index = Index::Constant(std::numeric_limits<IndexElement>::max());

  std::stack<std::pair<QuadtreeIndex, const Node<CellDataSpecialized>&>> stack;
  stack.template emplace(getInternalRootNodeIndex(), quadtree_.getRootNode());
  while (!stack.empty()) {
    const QuadtreeIndex internal_node_index = stack.top().first;
    const Node<CellDataSpecialized>& node = stack.top().second;
    stack.pop();

    if (node.hasChildrenArray()) {
      for (QuadtreeIndex::RelativeChild child_idx = 0;
           child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
        if (node.hasChild(child_idx)) {
          const QuadtreeIndex child_node_index =
              internal_node_index.computeChildIndex(child_idx);
          const Node<CellDataSpecialized>& child_node =
              *node.getChild(child_idx);
          stack.template emplace(child_node_index, child_node);
        }
      }
    } else if (OccupancyState::isObserved(node.data())) {
      const Index index = toExternalIndex(internal_node_index);
      min_index = min_index.cwiseMin(index);
    }
  }

  return min_index;
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential max candidates
template <typename CellT>
Index DifferencingQuadtree<CellT>::getMaxIndex() const {
  Index max_index =
      Index::Constant(std::numeric_limits<IndexElement>::lowest());

  std::stack<std::pair<QuadtreeIndex, const Node<CellDataSpecialized>&>> stack;
  stack.template emplace(getInternalRootNodeIndex(), quadtree_.getRootNode());
  while (!stack.empty()) {
    const QuadtreeIndex internal_node_index = stack.top().first;
    const Node<CellDataSpecialized>& node = stack.top().second;
    stack.pop();

    if (node.hasChildrenArray()) {
      for (QuadtreeIndex::RelativeChild child_idx = 0;
           child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
        if (node.hasChild(child_idx)) {
          const QuadtreeIndex child_node_index =
              internal_node_index.computeChildIndex(child_idx);
          const Node<CellDataSpecialized>& child_node =
              *node.getChild(child_idx);
          stack.template emplace(child_node_index, child_node);
        }
      }
    } else if (OccupancyState::isObserved(node.data())) {
      const Index index = toExternalIndex(internal_node_index);
      max_index = max_index.cwiseMax(index);
    }
  }

  return max_index;
}

template <typename CellT>
bool DifferencingQuadtree<CellT>::hasCell(const Index& index) const {
  const Node<CellDataSpecialized>* deepest_node_at_index =
      getDeepestNodeAtIndex(index);
  return deepest_node_at_index;
}

template <typename CellT>
FloatingPoint DifferencingQuadtree<CellT>::getCellValue(
    const Index& index) const {
  const QuadtreeIndex deepest_possible_node_index = toInternal(index);
  const std::vector<QuadtreeIndex::RelativeChild> child_indices =
      deepest_possible_node_index.computeRelativeChildIndices<kMaxHeight>();
  const Node<CellDataSpecialized>* node = &quadtree_.getRootNode();
  FloatingPoint value = node->data();
  for (const QuadtreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
    value += node->data();
  }
  return value;
}

template <typename CellT>
void DifferencingQuadtree<CellT>::setCellValue(const Index& index,
                                               FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const QuadtreeIndex internal_node_index = toInternal(index);
  Node<CellDataSpecialized>* node =
      quadtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
void DifferencingQuadtree<CellT>::setCellValue(const QuadtreeIndex& node_index,
                                               FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  Node<CellDataSpecialized>* node =
      quadtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

template <typename CellT>
void DifferencingQuadtree<CellT>::addToCellValue(const Index& index,
                                                 FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const QuadtreeIndex internal_node_index = toInternal(index);
  Node<CellDataSpecialized>* node =
      quadtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() += update;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
void DifferencingQuadtree<CellT>::addToCellValue(
    const QuadtreeIndex& node_index, FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  Node<CellDataSpecialized>* node =
      quadtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() += update;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

template <typename CellT>
void DifferencingQuadtree<CellT>::forEachLeaf(
    VolumetricDataStructure::IndexedLeafVisitorFunction visitor_fn) const {
  struct StackElement {
    const QuadtreeIndex internal_node_index;
    const Node<CellDataSpecialized>& node;
    const FloatingPoint parent_value = 0.f;
  };
  std::stack<StackElement> stack;

  stack.template emplace(
      StackElement{getInternalRootNodeIndex(), quadtree_.getRootNode(), 0.f});
  while (!stack.empty()) {
    const QuadtreeIndex internal_node_index = stack.top().internal_node_index;
    const Node<CellDataSpecialized>& node = stack.top().node;
    const FloatingPoint node_value = stack.top().parent_value + node.data();
    stack.pop();

    if (node.hasChildrenArray()) {
      for (QuadtreeIndex::RelativeChild child_idx = 0;
           child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
        const QuadtreeIndex internal_child_node_index =
            internal_node_index.computeChildIndex(child_idx);
        if (node.hasChild(child_idx)) {
          const Node<CellDataSpecialized>& child_node =
              *node.getChild(child_idx);
          stack.template emplace(
              StackElement{internal_child_node_index, child_node, node_value});
        } else {
          // Hallucinate the missing leaves
          // NOTE: This is necessary since the inner nodes in the data structure
          //       can overlap with each other and with leaves, but we want the
          //       visuals to be non-overlapping while still covering all
          //       observed space.
          const QuadtreeIndex node_index =
              toExternalNodeIndex(internal_child_node_index);
          visitor_fn(node_index, node_value);
        }
      }
    } else {
      const QuadtreeIndex node_index = toExternalNodeIndex(internal_node_index);
      visitor_fn(node_index, node_value);
    }
  }
}

template <typename CellT>
cv::Mat DifferencingQuadtree<CellT>::getImage(bool /*use_color*/) const {
  // TODO(victorr): Implement this
  return {};
}

template <typename CellT>
bool DifferencingQuadtree<CellT>::save(const std::string& /*file_path_prefix*/,
                                       bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool DifferencingQuadtree<CellT>::load(const std::string& /*file_path_prefix*/,
                                       bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
const Node<typename CellT::Specialized>*
DifferencingQuadtree<CellT>::getDeepestNodeAtIndex(const Index& index) const {
  const QuadtreeIndex deepest_possible_internal_node_index = toInternal(index);
  const Node<CellDataSpecialized>* node = &quadtree_.getRootNode();
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

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_DIFFERENCING_QUADTREE_INL_H_
