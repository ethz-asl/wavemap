#ifndef WAVEMAP_CORE_INDEXING_NDTREE_INDEX_H_
#define WAVEMAP_CORE_INDEXING_NDTREE_INDEX_H_

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

  //! The node's resolution level in the octree
  //! @note A height of 0 corresponds to the map’s maximum resolution.
  //!       The node's size is doubled each time the height is increased by 1.
  Element height = 0;
  //! The node's XYZ position in the octree’s grid at the resolution level set
  //! by *height*
  Position position = Position::Zero();

  bool operator==(const NdtreeIndex& other) const {
    return height == other.height && position == other.position;
  }
  bool operator!=(const NdtreeIndex& other) const {
    return !(*this == other);  // NOLINT
  }

  //! Compute the index of the node's direct parent
  NdtreeIndex computeParentIndex() const;
  //! Compute the index of the node's parent (or ancestor) at *parent_height*
  NdtreeIndex computeParentIndex(Element parent_height) const;

  //! Compute the index of the node's n-th child
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

#endif  // WAVEMAP_CORE_INDEXING_NDTREE_INDEX_H_
