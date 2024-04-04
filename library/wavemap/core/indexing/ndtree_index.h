#ifndef WAVEMAP_INDEXING_NDTREE_INDEX_H_
#define WAVEMAP_INDEXING_NDTREE_INDEX_H_

#include <limits>
#include <string>
#include <vector>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/math/int_math.h"

namespace wavemap {
using NdtreeIndexRelativeChild = uint8_t;

template <int dim>
struct NdtreeIndex {
  using Element = IndexElement;
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

  static IndexElement computeLastCommonAncestorHeight(MortonIndex first_morton,
                                                      Element first_height,
                                                      MortonIndex second_morton,
                                                      Element second_height);

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

#include "wavemap/core/indexing/impl/ndtree_index_inl.h"

#endif  // WAVEMAP_INDEXING_NDTREE_INDEX_H_
