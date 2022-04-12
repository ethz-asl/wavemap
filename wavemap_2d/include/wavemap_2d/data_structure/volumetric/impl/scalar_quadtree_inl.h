#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_SCALAR_QUADTREE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_SCALAR_QUADTREE_INL_H_

#include <limits>
#include <stack>
#include <string>
#include <utility>

#include "wavemap_2d/transform/tree/child_averaging.h"

namespace wavemap_2d {
template <typename CellT>
void ScalarQuadtree<CellT>::averageAndPrune() {
  for (auto& node :
       quadtree_.template getIterator<TraversalOrder::kDepthFirstPostorder>()) {
    AverageAndPruneChildren<UnboundedOccupancyCell::Specialized>(node);
  }
}

template <typename CellT>
Index ScalarQuadtree<CellT>::getMinPossibleIndex() const {
  return nodeIndexToIndex(NodeIndex{max_depth_, Index::Constant(0)});
}

template <typename CellT>
Index ScalarQuadtree<CellT>::getMaxPossibleIndex() const {
  return nodeIndexToIndex(NodeIndex{
      max_depth_, Index::Constant(constexpr_functions::exp2(max_depth_))});
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential min candidates
template <typename CellT>
Index ScalarQuadtree<CellT>::getMinIndex() const {
  Index min_index = Index::Constant(std::numeric_limits<IndexElement>::max());

  std::stack<std::pair<NodeIndex, const Node<CellDataSpecialized>*>> stack;
  stack.template emplace(NodeIndex{}, &quadtree_.getRootNode());
  while (!stack.empty()) {
    const NodeIndex node_index = stack.top().first;
    const Node<CellDataSpecialized>* node = stack.top().second;
    stack.pop();

    if (node->hasChildrenArray()) {
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
      const Index index = nodeIndexToIndex(node_index);
      min_index = min_index.cwiseMin(index);
    }
  }

  return min_index;
}

// TODO(victorr): Replace this with an implementation that only expands
//                potential max candidates
template <typename CellT>
Index ScalarQuadtree<CellT>::getMaxIndex() const {
  Index max_index =
      Index::Constant(std::numeric_limits<IndexElement>::lowest());

  std::stack<std::pair<NodeIndex, const Node<CellDataSpecialized>*>> stack;
  stack.template emplace(NodeIndex{}, &quadtree_.getRootNode());
  while (!stack.empty()) {
    const NodeIndex node_index = stack.top().first;
    const Node<CellDataSpecialized>* node = stack.top().second;
    stack.pop();

    if (node->hasChildrenArray()) {
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
      const Index index = nodeIndexToIndex(node_index);
      max_index = max_index.cwiseMax(index);
    }
  }

  return max_index;
}

template <typename CellT>
bool ScalarQuadtree<CellT>::hasCell(const Index& index) const {
  const NodeIndex node_index = indexToNodeIndex(index);
  const Node<CellDataSpecialized>* node = quadtree_.getNode(node_index);
  return node;
}

template <typename CellT>
FloatingPoint ScalarQuadtree<CellT>::getCellValue(const Index& index) const {
  const NodeIndex node_index = indexToNodeIndex(index);
  const Node<CellDataSpecialized>* node = quadtree_.getNode(node_index);
  if (node) {
    return node->data();
  } else {
    return 0.f;
  }
}

template <typename CellT>
void ScalarQuadtree<CellT>::setCellValue(const Index& index,
                                         FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const NodeIndex node_index = indexToNodeIndex(index);
  Node<CellDataSpecialized>* node =
      quadtree_.getNode(node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
void ScalarQuadtree<CellT>::addToCellValue(const Index& index,
                                           FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const NodeIndex node_index = indexToNodeIndex(index);
  Node<CellDataSpecialized>* node =
      quadtree_.getNode(node_index, kAutoAllocate);
  if (node) {
    node->data() = CellT::add(node->data(), update);
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << index;
  }
}

template <typename CellT>
cv::Mat ScalarQuadtree<CellT>::getImage(bool /*use_color*/) const {
  // TODO(victorr): Implement this
  return {};
}

template <typename CellT>
bool ScalarQuadtree<CellT>::save(const std::string& /*file_path_prefix*/,
                                 bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
bool ScalarQuadtree<CellT>::load(const std::string& /*file_path_prefix*/,
                                 bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}

template <typename CellT>
FloatingPoint ScalarQuadtree<CellT>::computeNodeWidthAtDepth(
    NodeIndexElement depth) {
  return root_node_width_ / std::exp2(depth);
}

template <typename CellT>
Vector ScalarQuadtree<CellT>::computeNodeHalvedDiagonalAtDepth(
    NodeIndexElement depth) {
  return Vector::Constant(0.5f) * computeNodeWidthAtDepth(depth);
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_SCALAR_QUADTREE_INL_H_
