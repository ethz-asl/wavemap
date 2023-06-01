#ifndef WAVEMAP_INDEXING_NDTREE_INDEX_H_
#define WAVEMAP_INDEXING_NDTREE_INDEX_H_

#include <limits>
#include <string>
#include <vector>

#include "wavemap/common.h"
#include "wavemap/utils/int_math.h"

namespace wavemap {
using NdtreeIndexElement = int;
using NdtreeIndexRelativeChild = uint8_t;

// TODO(victorr): Consider renaming NdtreeIndex to something like
//                HierarchicalNdIndex, since it can represent hierarchical
//                quadtrant/octant subvolumes regardless of the exact data
//                structure that's being indexed.
template <int dim>
struct NdtreeIndex {
  using Element = NdtreeIndexElement;
  using RelativeChild = NdtreeIndexRelativeChild;
  using Position = Eigen::Matrix<Element, dim, 1>;

  static constexpr Element kDim = dim;
  static constexpr RelativeChild kNumChildren = int_math::exp2(dim);
  using ChildArray = std::array<NdtreeIndex, kNumChildren>;

  Element height = 0;
  Position position = Position::Zero();

  bool operator==(const NdtreeIndex& other) const {
    return height == other.height && position == other.position;
  }
  bool operator!=(const NdtreeIndex& other) const {
    return !(*this == other);  // NOLINT
  }

  NdtreeIndex computeParentIndex() const;
  NdtreeIndex computeParentIndex(Element parent_height) const;

  NdtreeIndex computeChildIndex(RelativeChild relative_child_index) const;
  ChildArray computeChildIndices() const;

  RelativeChild computeRelativeChildIndex() const;
  static RelativeChild computeRelativeChildIndex(MortonIndex morton,
                                                 Element parent_height);

  static LinearIndex computeLevelTraversalDistance(MortonIndex morton,
                                                   Element parent_height,
                                                   Element child_height);
  static LinearIndex computeTreeTraversalDistance(MortonIndex morton,
                                                  Element parent_height,
                                                  Element child_height);

  std::string toString() const;
};

using BinaryTreeIndex = NdtreeIndex<1>;
using QuadtreeIndex = NdtreeIndex<2>;
using OctreeIndex = NdtreeIndex<3>;
}  // namespace wavemap

#include "wavemap/indexing/impl/ndtree_index_inl.h"

#endif  // WAVEMAP_INDEXING_NDTREE_INDEX_H_
