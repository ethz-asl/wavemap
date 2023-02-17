#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_DIFFERENCING_NDTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_DIFFERENCING_NDTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap {
template <typename CellT, int dim>
void VolumetricDifferencingNdtree<CellT, dim>::prune() {
  std::function<void(FloatingPoint, NodeType&)> recursive_fn =
      [&recursive_fn](FloatingPoint parent_value, NodeType& node) {
        // Process the children first
        const FloatingPoint node_value = parent_value + node.data();
        if (node.hasChildrenArray()) {
          for (NdtreeIndexRelativeChild child_idx = 0;
               child_idx < NdtreeIndex<dim>::kNumChildren; ++child_idx) {
            if (node.hasChild(child_idx)) {
              recursive_fn(node_value, *node.getChild(child_idx));
            }
          }
        }

        if (node.hasChildrenArray()) {
          // Compute the average of all children
          typename CellT::Specialized child_average{};
          for (int child_idx = 0; child_idx < NdtreeIndex<dim>::kNumChildren;
               ++child_idx) {
            NodeType* child = node.getChild(child_idx);
            if (child) {
              child_average += child->data();
            }
          }
          child_average /= static_cast<typename CellT::Specialized>(
              NdtreeIndex<dim>::kNumChildren);
          const bool child_average_is_non_zero =
              kEpsilon < std::abs(child_average);

          // Propagate the averages from the children to their parents
          if (child_average_is_non_zero) {
            node.data() += child_average;
          }

          // Subtract the average from the children and prune away zero leaves
          bool has_non_empty_child = false;
          for (int child_idx = 0; child_idx < NdtreeIndex<dim>::kNumChildren;
               ++child_idx) {
            NodeType* child = node.getChild(child_idx);
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

  recursive_fn(0.f, ndtree_.getRootNode());
}

template <typename CellT, int dim>
typename NdtreeIndex<dim>::ChildArray
VolumetricDifferencingNdtree<CellT, dim>::getFirstChildIndices() const {
  typename NdtreeIndex<dim>::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

template <typename CellT, int dim>
Index<dim> VolumetricDifferencingNdtree<CellT, dim>::getMinIndex() const {
  if (empty()) {
    return {};
  }

  Index<dim> min_index = getMaxPossibleIndex();
  forEachLeaf([&min_index](const NdtreeIndex<dim>& node_index,
                           FloatingPoint value) {
    if (OccupancyState::isObserved(value)) {
      const Index<dim> index = convert::nodeIndexToMinCornerIndex(node_index);
      min_index = min_index.cwiseMin(index);
    }
  });
  return min_index;
}

template <typename CellT, int dim>
Index<dim> VolumetricDifferencingNdtree<CellT, dim>::getMaxIndex() const {
  if (empty()) {
    return {};
  }

  Index<dim> max_index = getMinPossibleIndex();
  forEachLeaf([&max_index](const NdtreeIndex<dim>& node_index,
                           FloatingPoint value) {
    if (OccupancyState::isObserved(value)) {
      const Index<dim> index = convert::nodeIndexToMaxCornerIndex(node_index);
      max_index = max_index.cwiseMax(index);
    }
  });
  return max_index;
}

template <typename CellT, int dim>
Index<dim> VolumetricDifferencingNdtree<CellT, dim>::getMinPossibleIndex()
    const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT, int dim>
Index<dim> VolumetricDifferencingNdtree<CellT, dim>::getMaxPossibleIndex()
    const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT, int dim>
FloatingPoint VolumetricDifferencingNdtree<CellT, dim>::getCellValue(
    const Index<dim>& index) const {
  const NdtreeIndex<dim> deepest_possible_node_index = toInternal(index);
  const std::vector<NdtreeIndexRelativeChild> child_indices =
      deepest_possible_node_index.template computeRelativeChildIndices<
          VolumetricNdtreeInterface<dim>::kMaxHeight>();
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = node->data();
  for (const NdtreeIndexRelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
    value += node->data();
  }
  return value;
}

template <typename CellT, int dim>
void VolumetricDifferencingNdtree<CellT, dim>::setCellValue(
    const Index<dim>& index, FloatingPoint new_value) {
  const NdtreeIndex<dim> node_index =
      convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

template <typename CellT, int dim>
void VolumetricDifferencingNdtree<CellT, dim>::setCellValue(
    const NdtreeIndex<dim>& node_index, FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const NdtreeIndex<dim> internal_node_index = toInternal(node_index);
  NodeType* node = ndtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

template <typename CellT, int dim>
void VolumetricDifferencingNdtree<CellT, dim>::addToCellValue(
    const Index<dim>& index, FloatingPoint update) {
  const NdtreeIndex<dim> node_index =
      convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

template <typename CellT, int dim>
void VolumetricDifferencingNdtree<CellT, dim>::addToCellValue(
    const NdtreeIndex<dim>& node_index, FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const NdtreeIndex<dim> internal_node_index = toInternal(node_index);
  NodeType* node = ndtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() += update;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

template <typename CellT, int dim>
void VolumetricDifferencingNdtree<CellT, dim>::forEachLeaf(
    typename VolumetricDataStructureBase<dim>::IndexedLeafVisitorFunction
        visitor_fn) const {
  std::stack<StackElement> stack;
  stack.template emplace(
      StackElement{getInternalRootNodeIndex(), ndtree_.getRootNode(), 0.f});
  while (!stack.empty()) {
    const NdtreeIndex<dim> node_index = stack.top().node_index;
    const NodeType& node = stack.top().node;
    const FloatingPoint node_value = stack.top().parent_value + node.data();
    stack.pop();

    if (node.hasChildrenArray()) {
      for (NdtreeIndexRelativeChild child_idx = 0;
           child_idx < NdtreeIndex<dim>::kNumChildren; ++child_idx) {
        const NdtreeIndex<dim> child_node_index =
            node_index.computeChildIndex(child_idx);
        if (node.hasChild(child_idx)) {
          const NodeType& child_node = *node.getChild(child_idx);
          stack.template emplace(
              StackElement{child_node_index, child_node, node_value});
        } else {
          // Hallucinate the missing leaves
          // NOTE: This is necessary since the inner nodes in the data structure
          //       can overlap with each other and with leaves, but we want the
          //       visuals to be non-overlapping while still covering all
          //       observed space.
          const NdtreeIndex<dim> external_node_index =
              toExternalNodeIndex(child_node_index);
          visitor_fn(external_node_index, node_value);
        }
      }
    } else {
      const NdtreeIndex<dim> external_node_index =
          toExternalNodeIndex(node_index);
      visitor_fn(external_node_index, node_value);
    }
  }
}

template <typename CellT, int dim>
bool VolumetricDifferencingNdtree<CellT, dim>::save(
    const std::string& /*file_path_prefix*/,
    bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT, int dim>
bool VolumetricDifferencingNdtree<CellT, dim>::load(
    const std::string& /*file_path_prefix*/, bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_DIFFERENCING_NDTREE_INL_H_
