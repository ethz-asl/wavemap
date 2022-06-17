#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_TREE_INL_H_

#include <limits>
#include <string>
#include <vector>

namespace wavemap_2d {

void WaveletTree::prune() {
  // TODO(victorr): Implement this
}

Index WaveletTree::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

Index WaveletTree::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
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
  // TODO(victorr): Implement this
  return {};
}

Index WaveletTree::getMaxIndex() const {
  // TODO(victorr): Implement this
  return {};
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
  for (const QuadtreeIndex::RelativeChild child_index : child_indices) {
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
    QuadtreeIndex::RelativeChild relative_child_idx = child_indices[depth];
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
  for (const QuadtreeIndex::RelativeChild child_index : child_indices) {
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
    QuadtreeIndex::RelativeChild relative_child_idx = child_indices[depth];
    coefficients = HaarWaveletType::forwardSingleChild(coefficients.scale,
                                                       relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

void WaveletTree::forEachLeaf(
    VolumetricDataStructure::IndexedLeafVisitorFunction /*visitor_fn*/) const {
  // TODO(victorr): Implement this
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
