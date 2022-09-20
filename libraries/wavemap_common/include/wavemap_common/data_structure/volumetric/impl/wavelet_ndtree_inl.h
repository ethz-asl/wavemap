#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_NDTREE_INL_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_NDTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap_common/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap {
template <typename CellT, int dim>
void WaveletNdtree<CellT, dim>::prune() {
  std::function<typename WaveletNdtreeInterface<dim>::Coefficients::Scale(
      typename WaveletNdtreeInterface<dim>::NodeType&,
      typename WaveletNdtreeInterface<dim>::Coefficients::Scale)>
      recursive_fn = [&recursive_fn](
                         typename WaveletNdtreeInterface<dim>::NodeType& node,
                         typename WaveletNdtreeInterface<
                             dim>::Coefficients::Scale scale_coefficient) {
        typename WaveletNdtreeInterface<dim>::Coefficients::CoefficientsArray
            child_scale_coefficients =
                WaveletNdtreeInterface<dim>::Transform::backward(
                    {scale_coefficient, node.data()});

        bool has_at_least_one_child = false;
        for (typename NdtreeIndex<dim>::RelativeChild child_idx = 0;
             child_idx < NdtreeIndex<dim>::kNumChildren; ++child_idx) {
          if (node.hasChild(child_idx)) {
            typename WaveletNdtreeInterface<dim>::NodeType& child_node =
                *node.getChild(child_idx);
            child_scale_coefficients[child_idx] =
                recursive_fn(child_node, child_scale_coefficients[child_idx]);
            if (!child_node.hasChildrenArray() &&
                std::all_of(child_node.data().begin(), child_node.data().end(),
                            [](auto coefficient) {
                              return std::abs(coefficient) < 1e-4f;
                            })) {
              node.deleteChild(child_idx);
            } else {
              has_at_least_one_child = true;
            }
          } else {
            child_scale_coefficients[child_idx] -=
                CellType::threshold(child_scale_coefficients[child_idx]);
          }
        }
        if (!has_at_least_one_child) {
          node.deleteChildrenArray();
        }

        const auto [scale_update, detail_updates] =
            WaveletNdtreeInterface<dim>::Transform::forward(
                child_scale_coefficients);
        node.data() -= detail_updates;

        return scale_update;
      };

  root_scale_coefficient_ -=
      recursive_fn(ndtree_.getRootNode(), root_scale_coefficient_);
}

template <typename CellT, int dim>
void WaveletNdtree<CellT, dim>::clear() {
  ndtree_.clear();
  root_scale_coefficient_ = {};
}

template <typename CellT, int dim>
typename NdtreeIndex<dim>::ChildArray
WaveletNdtree<CellT, dim>::getFirstChildIndices() const {
  typename NdtreeIndex<dim>::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

template <typename CellT, int dim>
Index<dim> WaveletNdtree<CellT, dim>::getMinIndex() const {
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
Index<dim> WaveletNdtree<CellT, dim>::getMaxIndex() const {
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
Index<dim> WaveletNdtree<CellT, dim>::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT, int dim>
Index<dim> WaveletNdtree<CellT, dim>::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT, int dim>
FloatingPoint WaveletNdtree<CellT, dim>::getCellValue(
    const Index<dim>& index) const {
  const NdtreeIndex<dim> deepest_possible_node_index = toInternal(index);
  const std::vector<typename NdtreeIndex<dim>::RelativeChild> child_indices =
      deepest_possible_node_index.template computeRelativeChildIndices<
          VolumetricNdtreeInterface<dim>::kMaxHeight>();
  const typename WaveletNdtreeInterface<dim>::NodeType* node =
      &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (const typename NdtreeIndex<dim>::RelativeChild child_index :
       child_indices) {
    value = WaveletNdtreeInterface<dim>::Transform::backwardSingleChild(
        {value, node->data()}, child_index);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return value;
}

template <typename CellT, int dim>
void WaveletNdtree<CellT, dim>::setCellValue(const Index<dim>& index,
                                             FloatingPoint new_value) {
  const NdtreeIndex<dim> node_index =
      convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

template <typename CellT, int dim>
void WaveletNdtree<CellT, dim>::setCellValue(const NdtreeIndex<dim>& node_index,
                                             FloatingPoint new_value) {
  const NdtreeIndex<dim> internal_node_index = toInternal(node_index);
  const std::vector<typename NdtreeIndex<dim>::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricNdtreeInterface<dim>::kMaxHeight>();
  std::vector<typename WaveletNdtreeInterface<dim>::NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const typename NdtreeIndex<dim>::RelativeChild child_index =
        child_indices[depth];
    typename WaveletNdtreeInterface<dim>::NodeType* current_parent =
        node_ptrs.back();
    current_value = WaveletNdtreeInterface<dim>::Transform::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  typename WaveletNdtreeInterface<dim>::Coefficients::Parent coefficients{
      new_value - current_value, {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    const typename NdtreeIndex<dim>::RelativeChild relative_child_idx =
        child_indices[depth];
    typename WaveletNdtreeInterface<dim>::NodeType* current_node =
        node_ptrs[depth];
    coefficients = WaveletNdtreeInterface<dim>::Transform::forwardSingleChild(
        coefficients.scale, relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellT, int dim>
void WaveletNdtree<CellT, dim>::addToCellValue(const Index<dim>& index,
                                               FloatingPoint update) {
  const NdtreeIndex<dim> node_index =
      convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

template <typename CellT, int dim>
void WaveletNdtree<CellT, dim>::addToCellValue(
    const NdtreeIndex<dim>& node_index, FloatingPoint update) {
  const NdtreeIndex<dim> internal_node_index = toInternal(node_index);
  const std::vector<typename NdtreeIndex<dim>::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricNdtreeInterface<dim>::kMaxHeight>();
  std::vector<typename WaveletNdtreeInterface<dim>::NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const typename NdtreeIndex<dim>::RelativeChild child_index =
        child_indices[depth];
    typename WaveletNdtreeInterface<dim>::NodeType* current_parent =
        node_ptrs.back();
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  typename WaveletNdtreeInterface<dim>::Coefficients::Parent coefficients{
      update, {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    typename WaveletNdtreeInterface<dim>::NodeType* current_node =
        node_ptrs[depth];
    const typename NdtreeIndex<dim>::RelativeChild relative_child_idx =
        child_indices[depth];
    coefficients = WaveletNdtreeInterface<dim>::Transform::forwardSingleChild(
        coefficients.scale, relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellT, int dim>
void WaveletNdtree<CellT, dim>::forEachLeaf(
    typename VolumetricDataStructureBase<dim>::IndexedLeafVisitorFunction
        visitor_fn) const {
  if (empty()) {
    return;
  }

  std::stack<StackElement> stack;
  stack.template emplace(StackElement{getInternalRootNodeIndex(),
                                      ndtree_.getRootNode(),
                                      root_scale_coefficient_});
  while (!stack.empty()) {
    const NdtreeIndex<dim> node_index = stack.top().node_index;
    const typename WaveletNdtreeInterface<dim>::NodeType& node =
        stack.top().node;
    const FloatingPoint node_scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const typename WaveletNdtreeInterface<dim>::Coefficients::CoefficientsArray
        child_scale_coefficients =
            WaveletNdtreeInterface<dim>::Transform::backward(
                {node_scale_coefficient, {node.data()}});
    for (typename NdtreeIndex<dim>::RelativeChild child_idx = 0;
         child_idx < NdtreeIndex<dim>::kNumChildren; ++child_idx) {
      const NdtreeIndex<dim> child_node_index =
          node_index.computeChildIndex(child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];
      if (node.hasChild(child_idx)) {
        const typename WaveletNdtreeInterface<dim>::NodeType& child_node =
            *node.getChild(child_idx);
        stack.template emplace(StackElement{child_node_index, child_node,
                                            child_scale_coefficient});
      } else {
        const NdtreeIndex<dim> external_node_index =
            toExternalNodeIndex(child_node_index);
        visitor_fn(external_node_index, child_scale_coefficient);
      }
    }
  }
}

template <typename CellT, int dim>
typename WaveletNdtreeInterface<dim>::NodeType*
WaveletNdtree<CellT, dim>::getNode(const NdtreeIndex<dim>& node_index) {
  const NdtreeIndex<dim> internal_node_index = toInternal(node_index);
  const std::vector<typename NdtreeIndex<dim>::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricNdtreeInterface<dim>::kMaxHeight>();
  typename WaveletNdtreeInterface<dim>::NodeType* node = &ndtree_.getRootNode();
  for (const typename NdtreeIndex<dim>::RelativeChild child_index :
       child_indices) {
    if (!node->hasChild(child_index)) {
      node->template allocateChild(child_index);
    }
    node = node->getChild(child_index);
  }
  return node;
}

template <typename CellT, int dim>
const typename WaveletNdtreeInterface<dim>::NodeType*
WaveletNdtree<CellT, dim>::getNode(const NdtreeIndex<dim>& node_index) const {
  const NdtreeIndex<dim> internal_node_index = toInternal(node_index);
  const std::vector<typename NdtreeIndex<dim>::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricNdtreeInterface<dim>::kMaxHeight>();
  const typename WaveletNdtreeInterface<dim>::NodeType* node =
      &ndtree_.getRootNode();
  for (const typename NdtreeIndex<dim>::RelativeChild child_index :
       child_indices) {
    if (!node->hasChild(child_index)) {
      return nullptr;
    }
    node = node->getChild(child_index);
  }
  return node;
}

template <typename CellT, int dim>
bool WaveletNdtree<CellT, dim>::save(const std::string& /*file_path_prefix*/,
                                     bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT, int dim>
bool WaveletNdtree<CellT, dim>::load(const std::string& /*file_path_prefix*/,
                                     bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_NDTREE_INL_H_
