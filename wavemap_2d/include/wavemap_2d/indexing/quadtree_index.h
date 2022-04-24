#ifndef WAVEMAP_2D_INDEXING_QUADTREE_INDEX_H_
#define WAVEMAP_2D_INDEXING_QUADTREE_INDEX_H_

#include <string>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/utils/constexpr_functions.h"

namespace wavemap_2d {
using QuadtreeIndexElement = int;
using QuadtreeRelativeChildIndex = uint8_t;
using QuadtreePositionIndex =
    Eigen::Matrix<QuadtreeIndexElement, kMapDimension, 1>;

// TODO(victorr): Consider generalizing this to a templated n-dimensional
//                version supporting m-ary subdivision (not just diadic)
struct QuadtreeIndex {
  static constexpr int kNumChildren = constexpr_functions::exp2(kMapDimension);

  QuadtreeIndexElement depth = 0;
  QuadtreePositionIndex position = QuadtreePositionIndex::Zero();

  bool operator==(const QuadtreeIndex& other) const {
    return depth == other.depth && position == other.position;
  }
  bool operator!=(const QuadtreeIndex& other) const {
    return !(*this == other);
  }

  QuadtreeIndex computeParentIndex() const;
  QuadtreeIndex computeParentIndex(QuadtreeIndexElement parent_depth) const;
  std::vector<QuadtreeIndex> computeParentIndices() const;

  QuadtreeIndex computeChildIndex(
      QuadtreeRelativeChildIndex relative_child_index) const;
  std::vector<QuadtreeIndex> computeChildIndices() const;
  QuadtreeRelativeChildIndex computeRelativeChildIndex() const;
  std::vector<QuadtreeRelativeChildIndex> computeRelativeChildIndices() const;

  std::string toString() const;
};
}  // namespace wavemap_2d

#include "wavemap_2d/indexing/impl/quadtree_index_inl.h"

#endif  // WAVEMAP_2D_INDEXING_QUADTREE_INDEX_H_
