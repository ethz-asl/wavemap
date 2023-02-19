#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_OCTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_OCTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap {
template <typename CellT>
void WaveletOctree<CellT>::prune() {
  std::function<typename WaveletOctreeInterface::Coefficients::Scale(
      typename WaveletOctreeInterface::NodeType&,
      typename WaveletOctreeInterface::Coefficients::Scale)>
      recursive_fn = [&recursive_fn](
                         typename WaveletOctreeInterface::NodeType& node,
                         typename WaveletOctreeInterface::Coefficients::Scale
                             scale_coefficient) {
        typename WaveletOctreeInterface::Coefficients::CoefficientsArray
            child_scale_coefficients =
                WaveletOctreeInterface::Transform::backward(
                    {scale_coefficient, node.data()});

        bool has_at_least_one_child = false;
        for (typename OctreeIndex::RelativeChild child_idx = 0;
             child_idx < OctreeIndex::kNumChildren; ++child_idx) {
          if (node.hasChild(child_idx)) {
            typename WaveletOctreeInterface::NodeType& child_node =
                *node.getChild(child_idx);
            child_scale_coefficients[child_idx] =
                recursive_fn(child_node, child_scale_coefficients[child_idx]);
            if (!child_node.hasChildrenArray() &&
                std::all_of(child_node.data().cbegin(),
                            child_node.data().cend(), [](auto coefficient) {
                              return std::abs(coefficient) < 1e-3f;
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
            WaveletOctreeInterface::Transform::forward(
                child_scale_coefficients);
        node.data() -= detail_updates;

        return scale_update;
      };

  root_scale_coefficient_ -=
      recursive_fn(ndtree_.getRootNode(), root_scale_coefficient_);
}

template <typename CellT>
void WaveletOctree<CellT>::clear() {
  ndtree_.clear();
  root_scale_coefficient_ = {};
}

template <typename CellT>
typename OctreeIndex::ChildArray WaveletOctree<CellT>::getFirstChildIndices()
    const {
  typename OctreeIndex::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

template <typename CellT>
Index3D WaveletOctree<CellT>::getMinIndex() const {
  if (empty()) {
    return {};
  }

  Index3D min_index = getMaxPossibleIndex();
  forEachLeaf([&min_index](const OctreeIndex& node_index, FloatingPoint value) {
    if (OccupancyState::isObserved(value)) {
      const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
      min_index = min_index.cwiseMin(index);
    }
  });
  return min_index;
}

template <typename CellT>
Index3D WaveletOctree<CellT>::getMaxIndex() const {
  if (empty()) {
    return {};
  }

  Index3D max_index = getMinPossibleIndex();
  forEachLeaf([&max_index](const OctreeIndex& node_index, FloatingPoint value) {
    if (OccupancyState::isObserved(value)) {
      const Index3D index = convert::nodeIndexToMaxCornerIndex(node_index);
      max_index = max_index.cwiseMax(index);
    }
  });
  return max_index;
}

template <typename CellT>
Index3D WaveletOctree<CellT>::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT>
Index3D WaveletOctree<CellT>::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT>
FloatingPoint WaveletOctree<CellT>::getCellValue(const Index3D& index) const {
  const OctreeIndex deepest_possible_node_index = toInternal(index);
  const std::vector<typename OctreeIndex::RelativeChild> child_indices =
      deepest_possible_node_index.template computeRelativeChildIndices<
          VolumetricOctreeInterface::kMaxHeight>();
  const typename WaveletOctreeInterface::NodeType* node =
      &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (const typename OctreeIndex::RelativeChild child_index : child_indices) {
    value = WaveletOctreeInterface::Transform::backwardSingleChild(
        {value, node->data()}, child_index);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return value;
}

template <typename CellT>
void WaveletOctree<CellT>::setCellValue(const Index3D& index,
                                        FloatingPoint new_value) {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

template <typename CellT>
void WaveletOctree<CellT>::setCellValue(const OctreeIndex& node_index,
                                        FloatingPoint new_value) {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<typename OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricOctreeInterface::kMaxHeight>();
  std::vector<typename WaveletOctreeInterface::NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const typename OctreeIndex::RelativeChild child_index =
        child_indices[depth];
    typename WaveletOctreeInterface::NodeType* current_parent =
        node_ptrs.back();
    current_value = WaveletOctreeInterface::Transform::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  typename WaveletOctreeInterface::Coefficients::Parent coefficients{
      new_value - current_value, {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    const typename OctreeIndex::RelativeChild relative_child_idx =
        child_indices[depth];
    typename WaveletOctreeInterface::NodeType* current_node = node_ptrs[depth];
    coefficients = WaveletOctreeInterface::Transform::forwardSingleChild(
        coefficients.scale, relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellT>
void WaveletOctree<CellT>::addToCellValue(const Index3D& index,
                                          FloatingPoint update) {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

template <typename CellT>
void WaveletOctree<CellT>::addToCellValue(const OctreeIndex& node_index,
                                          FloatingPoint update) {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<typename OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricOctreeInterface::kMaxHeight>();
  std::vector<typename WaveletOctreeInterface::NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const typename OctreeIndex::RelativeChild child_index =
        child_indices[depth];
    typename WaveletOctreeInterface::NodeType* current_parent =
        node_ptrs.back();
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  typename WaveletOctreeInterface::Coefficients::Parent coefficients{update,
                                                                     {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    typename WaveletOctreeInterface::NodeType* current_node = node_ptrs[depth];
    const typename OctreeIndex::RelativeChild relative_child_idx =
        child_indices[depth];
    coefficients = WaveletOctreeInterface::Transform::forwardSingleChild(
        coefficients.scale, relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellT>
void WaveletOctree<CellT>::forEachLeaf(
    typename VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn)
    const {
  if (empty()) {
    return;
  }

  std::stack<StackElement> stack;
  stack.template emplace(StackElement{getInternalRootNodeIndex(),
                                      ndtree_.getRootNode(),
                                      root_scale_coefficient_});
  while (!stack.empty()) {
    const OctreeIndex node_index = stack.top().node_index;
    const typename WaveletOctreeInterface::NodeType& node = stack.top().node;
    const FloatingPoint node_scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const typename WaveletOctreeInterface::Coefficients::CoefficientsArray
        child_scale_coefficients = WaveletOctreeInterface::Transform::backward(
            {node_scale_coefficient, {node.data()}});
    for (typename OctreeIndex::RelativeChild child_idx = 0;
         child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index =
          node_index.computeChildIndex(child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];
      if (node.hasChild(child_idx)) {
        const typename WaveletOctreeInterface::NodeType& child_node =
            *node.getChild(child_idx);
        stack.template emplace(StackElement{child_node_index, child_node,
                                            child_scale_coefficient});
      } else {
        const OctreeIndex external_node_index =
            toExternalNodeIndex(child_node_index);
        visitor_fn(external_node_index, child_scale_coefficient);
      }
    }
  }
}

template <typename CellT>
typename WaveletOctreeInterface::NodeType* WaveletOctree<CellT>::getNode(
    const OctreeIndex& node_index) {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<typename OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricOctreeInterface::kMaxHeight>();
  typename WaveletOctreeInterface::NodeType* node = &ndtree_.getRootNode();
  for (const typename OctreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      node->template allocateChild(child_index);
    }
    node = node->getChild(child_index);
  }
  return node;
}

template <typename CellT>
const typename WaveletOctreeInterface::NodeType* WaveletOctree<CellT>::getNode(
    const OctreeIndex& node_index) const {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<typename OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<
          VolumetricOctreeInterface::kMaxHeight>();
  const typename WaveletOctreeInterface::NodeType* node =
      &ndtree_.getRootNode();
  for (const typename OctreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      return nullptr;
    }
    node = node->getChild(child_index);
  }
  return node;
}

template <typename CellT>
bool WaveletOctree<CellT>::save(const std::string& /*file_path_prefix*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool WaveletOctree<CellT>::load(const std::string& /*file_path_prefix*/) {
  // TODO(victorr): Implement this
  return false;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_OCTREE_INL_H_
