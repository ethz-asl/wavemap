#include "wavemap/data_structure/volumetric/wavelet_octree.h"

namespace wavemap {
void WaveletOctree::prune() {
  std::function<Coefficients::Scale(NodeType&, Coefficients::Scale)>
      recursive_fn = [&recursive_fn, this](
                         NodeType& node,
                         Coefficients::Scale scale_coefficient) {
        Coefficients::CoefficientsArray child_scale_coefficients =
            Transform::backward({scale_coefficient, node.data()});

        bool has_at_least_one_child = false;
        for (NdtreeIndexRelativeChild child_idx = 0;
             child_idx < OctreeIndex::kNumChildren; ++child_idx) {
          if (node.hasChild(child_idx)) {
            NodeType& child_node = *node.getChild(child_idx);
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
      };

  root_scale_coefficient_ -=
      recursive_fn(ndtree_.getRootNode(), root_scale_coefficient_);
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

  std::stack<StackElement> stack;
  stack.template emplace(StackElement{getInternalRootNodeIndex(),
                                      ndtree_.getRootNode(),
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
}  // namespace wavemap
