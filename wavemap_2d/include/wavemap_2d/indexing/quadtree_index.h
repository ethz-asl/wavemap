#ifndef WAVEMAP_2D_INDEXING_QUADTREE_INDEX_H_
#define WAVEMAP_2D_INDEXING_QUADTREE_INDEX_H_

#include <string>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/utils/constexpr_functions.h"

namespace wavemap_2d {
using NodeIndexElement = int;
using NodeRelativeChildIndex = uint8_t;
using NodePositionIndex = Eigen::Matrix<NodeIndexElement, kMapDimension, 1>;

// TODO(victorr): Consider generalizing this to a templated n-dimensional
//                version supporting m-ary subdivision (not just diadic)

// TODO(victorr): Update this class' name
struct NodeIndex {
  static constexpr int kNumChildren = constexpr_functions::exp2(kMapDimension);

  NodeIndexElement depth = 0;
  NodePositionIndex position = NodePositionIndex::Zero();

  bool operator==(const NodeIndex& other) const {
    return depth == other.depth && position == other.position;
  }
  bool operator!=(const NodeIndex& other) const { return !(*this == other); }

  NodeIndex computeParentIndex() const;
  NodeIndex computeParentIndex(NodeIndexElement parent_depth) const;
  std::vector<NodeIndex> computeParentIndices() const;

  NodeIndex computeChildIndex(
      NodeRelativeChildIndex relative_child_index) const;
  std::vector<NodeIndex> computeChildIndices() const;
  NodeRelativeChildIndex computeRelativeChildIndex() const;
  std::vector<NodeRelativeChildIndex> computeRelativeChildIndices() const;

  std::string toString() const;
};
}  // namespace wavemap_2d

#include "wavemap_2d/indexing/impl/quadtree_index_inl.h"

#endif  // WAVEMAP_2D_INDEXING_QUADTREE_INDEX_H_