#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_NDTREE_INL_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_NDTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap_common/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap {
template <typename CellT, int dim>
void VolumetricNdtree<CellT, dim>::prune() {
  std::function<void(FloatingPoint, NodeType&)> recursive_fn =
      [&recursive_fn](FloatingPoint parent_value, NodeType& node) {
        // Process the children first
        const FloatingPoint node_value = parent_value + node.data();
        if (node.hasChildrenArray()) {
          for (typename NdtreeIndex<dim>::RelativeChild child_idx = 0;
               child_idx < NdtreeIndex<dim>::kNumChildren; ++child_idx) {
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
          typename CellT::Specialized first_child_value = CellT::add(
              node_value, node.getChild(0) ? node.getChild(0)->data() : 0.f);
          for (typename NdtreeIndex<dim>::RelativeChild child_idx = 0;
               child_idx < NdtreeIndex<dim>::kNumChildren; ++child_idx) {
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
            node.data() = {};
          }
        } else {
          // Threshold the leaf values
          node.data() = CellT::threshold(node_value);
        }
      };

  recursive_fn(0.f, ndtree_.getRootNode());
}

template <typename CellT, int dim>
typename NdtreeIndex<dim>::ChildArray
VolumetricNdtree<CellT, dim>::getFirstChildIndices() const {
  typename NdtreeIndex<dim>::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

template <typename CellT, int dim>
Index<dim> VolumetricNdtree<CellT, dim>::getMinIndex() const {
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
Index<dim> VolumetricNdtree<CellT, dim>::getMaxIndex() const {
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
Index<dim> VolumetricNdtree<CellT, dim>::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT, int dim>
Index<dim> VolumetricNdtree<CellT, dim>::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT, int dim>
FloatingPoint VolumetricNdtree<CellT, dim>::getCellValue(
    const Index<dim>& index) const {
  const NodeType* deepest_node_at_index = getDeepestNodeAtIndex(index);
  if (deepest_node_at_index) {
    return deepest_node_at_index->data();
  }
  return 0.f;
}

template <typename CellT, int dim>
void VolumetricNdtree<CellT, dim>::setCellValue(const Index<dim>& index,
                                                FloatingPoint new_value) {
  const NdtreeIndex<dim> node_index =
      convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

template <typename CellT, int dim>
void VolumetricNdtree<CellT, dim>::setCellValue(
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
void VolumetricNdtree<CellT, dim>::addToCellValue(const Index<dim>& index,
                                                  FloatingPoint update) {
  const NdtreeIndex<dim> node_index =
      convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

template <typename CellT, int dim>
void VolumetricNdtree<CellT, dim>::addToCellValue(
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
void VolumetricNdtree<CellT, dim>::forEachLeaf(
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
      for (typename NdtreeIndex<dim>::RelativeChild child_idx = 0;
           child_idx < NdtreeIndex<dim>::kNumChildren; ++child_idx) {
        const NdtreeIndex<dim> child_node_index =
            node_index.computeChildIndex(child_idx);
        if (node.hasChild(child_idx)) {
          const NodeType& child_node = *node.getChild(child_idx);
          stack.template emplace(
              StackElement{child_node_index, child_node, node_value});
        } else {
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
bool VolumetricNdtree<CellT, dim>::save(const std::string& /*file_path_prefix*/,
                                        bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT, int dim>
bool VolumetricNdtree<CellT, dim>::load(const std::string& /*file_path_prefix*/,
                                        bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT, int dim>
const typename VolumetricNdtree<CellT, dim>::NodeType*
VolumetricNdtree<CellT, dim>::getDeepestNodeAtIndex(
    const Index<dim>& index) const {
  const NdtreeIndex<dim> deepest_possible_internal_node_index =
      toInternal(index);
  const NodeType* node = &ndtree_.getRootNode();
  const std::vector<typename NdtreeIndex<dim>::RelativeChild> child_indices =
      deepest_possible_internal_node_index.template computeRelativeChildIndices<
          VolumetricNdtreeInterface<dim>::kMaxHeight>();
  for (const typename NdtreeIndex<dim>::RelativeChild child_index :
       child_indices) {
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return node;
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_NDTREE_INL_H_
