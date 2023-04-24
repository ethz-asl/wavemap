#include "wavemap/data_structure/volumetric/wavelet_octree.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(WaveletOctreeConfig, (min_cell_width, SiUnit::kMeters),
                       (min_log_odds), (max_log_odds), (tree_height));

bool WaveletOctreeConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_cell_width, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_log_odds, max_log_odds, verbose);
  is_valid &= IS_PARAM_GT(tree_height, 0, verbose);

  return is_valid;
}

void WaveletOctree::threshold() {
  root_scale_coefficient_ -=
      recursiveThreshold(ndtree_.getRootNode(), root_scale_coefficient_);
}

void WaveletOctree::prune() {
  root_scale_coefficient_ -=
      recursivePrune(ndtree_.getRootNode(), root_scale_coefficient_);
}

Index3D WaveletOctree::getMinIndex() const {
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

Index3D WaveletOctree::getMaxIndex() const {
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

void WaveletOctree::forEachLeaf(
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  if (empty()) {
    return;
  }

  struct StackElement {
    const OctreeIndex node_index;
    const NodeType& node;
    const Coefficients::Scale scale_coefficient{};
  };
  std::stack<StackElement> stack;
  stack.emplace(StackElement{getInternalRootNodeIndex(), ndtree_.getRootNode(),
                             root_scale_coefficient_});

  while (!stack.empty()) {
    const OctreeIndex node_index = stack.top().node_index;
    const NodeType& node = stack.top().node;
    const FloatingPoint node_scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const Coefficients::CoefficientsArray child_scale_coefficients =
        Transform::backward({node_scale_coefficient, {node.data()}});
    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index =
          node_index.computeChildIndex(child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];
      if (node.hasChild(child_idx)) {
        const NodeType& child_node = *node.getChild(child_idx);
        stack.emplace(StackElement{child_node_index, child_node,
                                   child_scale_coefficient});
      } else {
        const OctreeIndex external_node_index =
            toExternalNodeIndex(child_node_index);
        visitor_fn(external_node_index, child_scale_coefficient);
      }
    }
  }
}

WaveletOctree::Coefficients::Scale WaveletOctree::recursiveThreshold(  // NOLINT
    WaveletOctree::NodeType& node, FloatingPoint scale_coefficient) {
  Coefficients::CoefficientsArray child_scale_coefficients =
      Transform::backward({scale_coefficient, node.data()});

  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (node.hasChild(child_idx)) {
      NodeType& child_node = *node.getChild(child_idx);
      child_scale_coefficients[child_idx] =
          recursiveThreshold(child_node, child_scale_coefficients[child_idx]);
    } else {
      child_scale_coefficients[child_idx] -=
          clamp(child_scale_coefficients[child_idx]);
    }
  }

  const auto [scale_update, detail_updates] =
      Transform::forward(child_scale_coefficients);
  node.data() -= detail_updates;

  return scale_update;
}

WaveletOctree::Coefficients::Scale WaveletOctree::recursivePrune(  // NOLINT
    WaveletOctree::NodeType& node, FloatingPoint scale_coefficient) {
  Coefficients::CoefficientsArray child_scale_coefficients =
      Transform::backward({scale_coefficient, node.data()});

  bool has_at_least_one_child = false;
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (node.hasChild(child_idx)) {
      NodeType& child_node = *node.getChild(child_idx);
      child_scale_coefficients[child_idx] =
          recursivePrune(child_node, child_scale_coefficients[child_idx]);
      if (!child_node.hasChildrenArray() && !child_node.hasNonzeroData(1e-3f)) {
        node.deleteChild(child_idx);
      } else {
        has_at_least_one_child = true;
      }
    } else {
      child_scale_coefficients[child_idx] -=
          clamp(child_scale_coefficients[child_idx]);
    }
  }
  if (!has_at_least_one_child) {
    node.deleteChildrenArray();
  }

  const auto [scale_update, detail_updates] =
      Transform::forward(child_scale_coefficients);
  node.data() -= detail_updates;

  return scale_update;
}
}  // namespace wavemap
