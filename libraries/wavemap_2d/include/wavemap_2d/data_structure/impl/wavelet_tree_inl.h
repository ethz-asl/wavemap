#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/data_structure/cell_types/occupancy_state.h"

namespace wavemap {
template <typename CellT>
void WaveletTree<CellT>::prune() {
  std::function<ScaleCoefficient(NodeType&, ScaleCoefficient)> recursive_fn =
      [&recursive_fn](NodeType& node, ScaleCoefficient scale_coefficient) {
        ChildScaleCoefficients child_scale_coefficients =
            HaarWaveletType::backward({scale_coefficient, node.data()});

        bool has_at_least_one_child = false;
        for (QuadtreeIndex::RelativeChild child_idx = 0;
             child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
          if (node.hasChild(child_idx)) {
            NodeType& child_node = *node.getChild(child_idx);
            child_scale_coefficients[child_idx] =
                recursive_fn(child_node, child_scale_coefficients[child_idx]);
            constexpr FloatingPoint kNegligibleDetailCoefficient = 1e-4f;
            if (!child_node.hasChildrenArray() &&
                std::abs(child_node.data().xx) < kNegligibleDetailCoefficient &&
                std::abs(child_node.data().yy) < kNegligibleDetailCoefficient &&
                std::abs(child_node.data().xy) < kNegligibleDetailCoefficient) {
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
            HaarWaveletType::forward(child_scale_coefficients);
        node.data() -= detail_updates;

        return scale_update;
      };

  root_scale_coefficient_ -=
      recursive_fn(quadtree_.getRootNode(), root_scale_coefficient_);
}

template <typename CellT>
void WaveletTree<CellT>::clear() {
  quadtree_.clear();
  root_scale_coefficient_ = {};
}

template <typename CellT>
QuadtreeIndex::ChildArray WaveletTree<CellT>::getFirstChildIndices() const {
  QuadtreeIndex::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

template <typename CellT>
Index WaveletTree<CellT>::getMinIndex() const {
  if (empty()) {
    return {};
  }

  Index min_index = getMaxPossibleIndex();
  forEachLeaf(
      [&min_index](const QuadtreeIndex& node_index, FloatingPoint value) {
        if (OccupancyState::isObserved(value)) {
          const Index index = convert::nodeIndexToMinCornerIndex(node_index);
          min_index = min_index.cwiseMin(index);
        }
      });
  return min_index;
}

template <typename CellT>
Index WaveletTree<CellT>::getMaxIndex() const {
  if (empty()) {
    return {};
  }

  Index max_index = getMinPossibleIndex();
  forEachLeaf(
      [&max_index](const QuadtreeIndex& node_index, FloatingPoint value) {
        if (OccupancyState::isObserved(value)) {
          const Index index = convert::nodeIndexToMaxCornerIndex(node_index);
          max_index = max_index.cwiseMax(index);
        }
      });
  return max_index;
}

template <typename CellT>
Index WaveletTree<CellT>::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT>
Index WaveletTree<CellT>::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

template <typename CellT>
FloatingPoint WaveletTree<CellT>::getCellValue(const Index& index) const {
  const QuadtreeIndex deepest_possible_node_index = toInternal(index);
  const std::vector<QuadtreeIndex::RelativeChild> child_indices =
      deepest_possible_node_index.computeRelativeChildIndices<kMaxHeight>();
  const NodeType* node = &quadtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (const QuadtreeIndex::RelativeChild child_index : child_indices) {
    value = HaarWaveletType::backwardSingleChild({value, node->data()},
                                                 child_index);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return value;
}

template <typename CellT>
void WaveletTree<CellT>::setCellValue(const Index& index,
                                      FloatingPoint new_value) {
  const QuadtreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

template <typename CellT>
void WaveletTree<CellT>::setCellValue(const QuadtreeIndex& node_index,
                                      FloatingPoint new_value) {
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  const std::vector<QuadtreeIndex::RelativeChild> child_indices =
      internal_node_index.computeRelativeChildIndices<kMaxHeight>();
  std::vector<NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&quadtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const QuadtreeIndex::RelativeChild child_index = child_indices[depth];
    NodeType* current_parent = node_ptrs.back();
    current_value = HaarWaveletType::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  ParentCoefficients coefficients{new_value - current_value, {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    const QuadtreeIndex::RelativeChild relative_child_idx =
        child_indices[depth];
    NodeType* current_node = node_ptrs[depth];
    coefficients = HaarWaveletType::forwardSingleChild(coefficients.scale,
                                                       relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellT>
void WaveletTree<CellT>::addToCellValue(const Index& index,
                                        FloatingPoint update) {
  const QuadtreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

template <typename CellT>
void WaveletTree<CellT>::addToCellValue(const QuadtreeIndex& node_index,
                                        FloatingPoint update) {
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  const std::vector<QuadtreeIndex::RelativeChild> child_indices =
      internal_node_index.computeRelativeChildIndices<kMaxHeight>();
  std::vector<NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&quadtree_.getRootNode());
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const QuadtreeIndex::RelativeChild child_index = child_indices[depth];
    NodeType* current_parent = node_ptrs.back();
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  ParentCoefficients coefficients{update, {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    NodeType* current_node = node_ptrs[depth];
    const QuadtreeIndex::RelativeChild relative_child_idx =
        child_indices[depth];
    coefficients = HaarWaveletType::forwardSingleChild(coefficients.scale,
                                                       relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellT>
void WaveletTree<CellT>::forEachLeaf(
    VolumetricDataStructure::IndexedLeafVisitorFunction visitor_fn) const {
  if (empty()) {
    return;
  }

  std::stack<StackElement> stack;
  stack.template emplace(StackElement{getInternalRootNodeIndex(),
                                      quadtree_.getRootNode(),
                                      root_scale_coefficient_});
  while (!stack.empty()) {
    const QuadtreeIndex node_index = stack.top().node_index;
    const NodeType& node = stack.top().node;
    const FloatingPoint node_scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const ChildScaleCoefficients child_scale_coefficients =
        HaarWaveletType::backward({node_scale_coefficient, {node.data()}});
    for (QuadtreeIndex::RelativeChild child_idx = 0;
         child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
      const QuadtreeIndex child_node_index =
          node_index.computeChildIndex(child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];
      if (node.hasChild(child_idx)) {
        const NodeType& child_node = *node.getChild(child_idx);
        stack.template emplace(StackElement{child_node_index, child_node,
                                            child_scale_coefficient});
      } else {
        const QuadtreeIndex external_node_index =
            toExternalNodeIndex(child_node_index);
        visitor_fn(external_node_index, child_scale_coefficient);
      }
    }
  }
}

template <typename CellT>
typename WaveletTree<CellT>::NodeType* WaveletTree<CellT>::getNode(
    const QuadtreeIndex& node_index) {
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  const std::vector<QuadtreeIndex::RelativeChild> child_indices =
      internal_node_index.computeRelativeChildIndices<kMaxHeight>();
  NodeType* node = &quadtree_.getRootNode();
  for (const QuadtreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      node->template allocateChild(child_index);
    }
    node = node->getChild(child_index);
  }
  return node;
}

template <typename CellT>
const typename WaveletTree<CellT>::NodeType* WaveletTree<CellT>::getNode(
    const QuadtreeIndex& node_index) const {
  const QuadtreeIndex internal_node_index = toInternal(node_index);
  const std::vector<QuadtreeIndex::RelativeChild> child_indices =
      internal_node_index.computeRelativeChildIndices<kMaxHeight>();
  const NodeType* node = &quadtree_.getRootNode();
  for (const QuadtreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      return nullptr;
    }
    node = node->getChild(child_index);
  }
  return node;
}

template <typename CellT>
cv::Mat WaveletTree<CellT>::getImage(bool /*use_color*/) const {
  // TODO(victorr): Implement this
  return {};
}

template <typename CellT>
bool WaveletTree<CellT>::save(const std::string& /*file_path_prefix*/,
                              bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool WaveletTree<CellT>::load(const std::string& /*file_path_prefix*/,
                              bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_
