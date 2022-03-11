#ifndef WAVEMAP_2D_TRANSFORM_TREE_CHILD_AVERAGING_H_
#define WAVEMAP_2D_TRANSFORM_TREE_CHILD_AVERAGING_H_

#include "wavemap_2d/datastructure/quadtree/node.h"

namespace wavemap_2d {
template <typename NodeDataType>
void AverageAndPruneChildren(Node<NodeDataType>& node) {
  if (node.hasChildrenArray()) {
    // Compute the average of all children
    auto child_sum = static_cast<NodeDataType>(0);
    for (int child_idx = 0; child_idx < NodeIndex::kNumChildren; ++child_idx) {
      Node<NodeDataType>* child_ptr = node.getChild(child_idx);
      if (child_ptr) {
        child_sum += child_ptr->data();
      } else {
        child_sum += node.data();
      }
    }
    node.data() =
        child_sum / static_cast<NodeDataType>(NodeIndex::kNumChildren);

    // Prune away children whose value is identical to the parent node
    bool has_non_empty_child = false;
    for (int child_idx = 0; child_idx < NodeIndex::kNumChildren; ++child_idx) {
      const Node<NodeDataType>* child_ptr = node.getChild(child_idx);
      if (child_ptr) {
        if ((std::abs(child_ptr->data() - node.data()) < kEpsilon) &&
            !child_ptr->hasChildrenArray()) {
          node.deleteChild(child_idx);
        } else {
          has_non_empty_child = true;
        }
      }
    }

    // Free up the children array if it only contains null ptrs
    if (!has_non_empty_child) {
      node.deleteChildrenArray();
    }
  }
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_TRANSFORM_TREE_CHILD_AVERAGING_H_
