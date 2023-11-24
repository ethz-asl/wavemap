#include "wavemap/map/volumetric_octree.h"

#include "wavemap/utils/query/occupancy_classifier.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(VolumetricOctreeConfig,
                      (min_cell_width)
                      (min_log_odds)
                      (max_log_odds)
                      (tree_height));

bool VolumetricOctreeConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_cell_width, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_log_odds, max_log_odds, verbose);
  is_valid &= IS_PARAM_GT(tree_height, 0, verbose);

  return is_valid;
}

void VolumetricOctree::threshold() {
  LOG(WARNING) << "Pure thresholding is not implemented for VolumetricOctrees. "
                  "Will prune instead.";
  prune();
}

void VolumetricOctree::prune() {
  std::function<void(FloatingPoint, NodeType&)> recursive_fn =
      [&recursive_fn, this](FloatingPoint parent_value, NodeType& node) {
        // Process the children first
        const FloatingPoint node_value = parent_value + node.data();
        if (node.hasChildrenArray()) {
          for (NdtreeIndexRelativeChild child_idx = 0;
               child_idx < OctreeIndex::kNumChildren; ++child_idx) {
            NodeType* child = node.getChild(child_idx);
            if (child) {
              recursive_fn(node_value, *child);
            } else if (kEpsilon < std::abs(node_value)) {
              // Always propagate non-zero internal node value down to leaves
              recursive_fn(node_value, node.getOrAllocateChild(child_idx));
            }
          }
        }

        if (node.hasChildrenArray()) {
          // Check whether the node's children are all identical leaves
          bool all_children_are_identical_leaves = true;
          FloatingPoint first_child_value = clampedAdd(
              node_value, node.getChild(0) ? node.getChild(0)->data() : 0.f);
          for (NdtreeIndexRelativeChild child_idx = 0;
               child_idx < OctreeIndex::kNumChildren; ++child_idx) {
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
          node.data() = clamp(node_value);
        }
      };

  recursive_fn(0.f, ndtree_.getRootNode());
}

Index3D VolumetricOctree::getMinIndex() const {
  if (empty()) {
    return {};
  }

  Index3D min_index = getMaxPossibleIndex();
  forEachLeaf([&min_index](const OctreeIndex& node_index, FloatingPoint value) {
    if (OccupancyClassifier::isObserved(value)) {
      const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
      min_index = min_index.cwiseMin(index);
    }
  });
  return min_index;
}

Index3D VolumetricOctree::getMaxIndex() const {
  if (empty()) {
    return {};
  }

  Index3D max_index = getMinPossibleIndex();
  forEachLeaf([&max_index](const OctreeIndex& node_index, FloatingPoint value) {
    if (OccupancyClassifier::isObserved(value)) {
      const Index3D index = convert::nodeIndexToMaxCornerIndex(node_index);
      max_index = max_index.cwiseMax(index);
    }
  });
  return max_index;
}

void VolumetricOctree::forEachLeaf(
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  struct StackElement {
    const OctreeIndex node_index;
    const NodeType& node;
    const FloatingPoint parent_value{};
  };
  std::stack<StackElement> stack;
  stack.emplace(
      StackElement{getInternalRootNodeIndex(), ndtree_.getRootNode(), 0.f});

  while (!stack.empty()) {
    const OctreeIndex node_index = stack.top().node_index;
    const NodeType& node = stack.top().node;
    const FloatingPoint node_value = stack.top().parent_value + node.data();
    stack.pop();
    if (node.hasChildrenArray()) {
      for (NdtreeIndexRelativeChild child_idx = 0;
           child_idx < OctreeIndex::kNumChildren; ++child_idx) {
        const OctreeIndex child_node_index =
            node_index.computeChildIndex(child_idx);
        if (node.hasChild(child_idx)) {
          const NodeType& child_node = *node.getChild(child_idx);
          stack.emplace(StackElement{child_node_index, child_node, node_value});
        } else {
          const OctreeIndex external_node_index =
              toExternalNodeIndex(child_node_index);
          visitor_fn(external_node_index, node_value);
        }
      }
    } else {
      const OctreeIndex external_node_index = toExternalNodeIndex(node_index);
      visitor_fn(external_node_index, node_value);
    }
  }
}
}  // namespace wavemap
