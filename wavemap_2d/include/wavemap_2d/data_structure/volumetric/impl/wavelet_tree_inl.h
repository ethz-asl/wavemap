#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

namespace wavemap_2d {

void WaveletTree::prune() {
  // TODO(victorr): Finish testing/debugging this
  std::function<HaarWaveletType::Coefficients::Scale(
      HaarWaveletType::Coefficients::Scale, NodeType&)>
      recursive_fn = [&recursive_fn](
                         HaarWaveletType::Coefficients::Scale scale_coefficient,
                         NodeType& node) {
        const HaarWaveletType::ChildScaleCoefficients child_scale_coefficients =
            HaarWaveletType::backward({scale_coefficient, node.data()});
        HaarWaveletType::ChildScaleCoefficients
            child_scale_coefficient_updates{};

        if (node.hasChildrenArray()) {
          bool has_at_least_one_child = false;
          for (QuadtreeIndex::RelativeChild child_idx = 0;
               child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
            if (node.hasChild(child_idx)) {
              NodeType& child_node = *node.getChild(child_idx);
              const HaarWaveletType::Coefficients::Scale
                  child_scale_coefficient = child_scale_coefficients[child_idx];
              child_scale_coefficient_updates[child_idx] =
                  recursive_fn(child_scale_coefficient, child_node);
              if (!child_node.hasChildrenArray() &&
                  std::abs(child_node.data().xx) < kEpsilon &&
                  std::abs(child_node.data().yy) < kEpsilon &&
                  std::abs(child_node.data().xy) < kEpsilon) {
                node.deleteChild(child_idx);
              } else {
                has_at_least_one_child = true;
              }
            }
          }
          if (!has_at_least_one_child) {
            node.deleteChildrenArray();
          }
        } else {
          for (QuadtreeIndex::RelativeChild child_idx = 0;
               child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
            const HaarWaveletType::Coefficients::Scale child_scale_coefficient =
                child_scale_coefficients[child_idx];
            child_scale_coefficient_updates[child_idx] =
                child_scale_coefficient -
                CellType::threshold(child_scale_coefficient);
          }
        }

        const HaarWaveletType::ParentCoefficients child_coefficients_update =
            HaarWaveletType::forward(child_scale_coefficient_updates);
        node.data() -= child_coefficients_update.details;

        return child_coefficients_update.scale;
      };

  root_scale_coefficient_ -=
      recursive_fn(root_scale_coefficient_, quadtree_.getRootNode());
}

QuadtreeIndex::ChildArray WaveletTree::getFirstChildIndices() const {
  QuadtreeIndex::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

Index WaveletTree::getMinIndex() const {
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

Index WaveletTree::getMaxIndex() const {
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

Index WaveletTree::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

Index WaveletTree::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

FloatingPoint WaveletTree::getCellValue(const Index& index) const {
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

void WaveletTree::setCellValue(const Index& index, FloatingPoint new_value) {
  const QuadtreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

void WaveletTree::setCellValue(const QuadtreeIndex& node_index,
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

  HaarWaveletType::ParentCoefficients coefficients{new_value - current_value,
                                                   {}};
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

void WaveletTree::addToCellValue(const Index& index, FloatingPoint update) {
  const QuadtreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

void WaveletTree::addToCellValue(const QuadtreeIndex& node_index,
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

  HaarWaveletType::ParentCoefficients coefficients{update, {}};
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

void WaveletTree::forEachLeaf(
    VolumetricDataStructure::IndexedLeafVisitorFunction visitor_fn) const {
  std::stack<StackElement> stack;
  stack.template emplace(StackElement{getInternalRootNodeIndex(),
                                      quadtree_.getRootNode(),
                                      root_scale_coefficient_});
  while (!stack.empty()) {
    const QuadtreeIndex internal_node_index = stack.top().internal_node_index;
    const NodeType& node = stack.top().node;
    const FloatingPoint node_scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    if (node.hasChildrenArray()) {
      const HaarWaveletType::ChildScaleCoefficients child_scale_coefficients =
          HaarWaveletType::backward({node_scale_coefficient, {node.data()}});
      for (QuadtreeIndex::RelativeChild child_idx = 0;
           child_idx < QuadtreeIndex::kNumChildren; ++child_idx) {
        const QuadtreeIndex internal_child_node_index =
            internal_node_index.computeChildIndex(child_idx);
        const FloatingPoint child_scale_coefficient =
            child_scale_coefficients[child_idx];
        if (node.hasChild(child_idx)) {
          const NodeType& child_node = *node.getChild(child_idx);
          stack.template emplace(StackElement{
              internal_child_node_index, child_node, child_scale_coefficient});
        } else {
          // Hallucinate the missing leaves
          // NOTE: This is necessary since the inner nodes in the data structure
          //       can overlap with each other and with leaves, but we want the
          //       visuals to be non-overlapping while still covering all
          //       observed space.
          const QuadtreeIndex node_index =
              toExternalNodeIndex(internal_child_node_index);
          visitor_fn(node_index, child_scale_coefficient);
        }
      }
    } else {
      const QuadtreeIndex node_index = toExternalNodeIndex(internal_node_index);
      visitor_fn(node_index, node_scale_coefficient);
    }
  }
}

cv::Mat WaveletTree::getImage(bool /*use_color*/) const {
  // TODO(victorr): Implement this
  return {};
}

bool WaveletTree::save(const std::string& /*file_path_prefix*/,
                       bool /*use_floating_precision*/) const {
  // TODO(victorr): Implement this
  return false;
}

bool WaveletTree::load(const std::string& /*file_path_prefix*/,
                       bool /*used_floating_precision*/) {
  // TODO(victorr): Implement this
  return false;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_
