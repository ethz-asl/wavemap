#ifndef WAVEMAP_COMMON_INDEXING_NDTREE_INDEX_H_
#define WAVEMAP_COMMON_INDEXING_NDTREE_INDEX_H_

#include <string>
#include <vector>

#include "wavemap_common/common.h"
#include "wavemap_common/utils/int_math.h"

namespace wavemap {
// TODO(victorr): Consider renaming NdtreeIndex to something like
//                HierarchicalNdIndex, since it can represent hierarchical
//                quadtrant/octant subvolumes regardless of the exact data
//                structure that's being indexed.
template <int dim>
struct NdtreeIndex {
  using Element = int;
  using RelativeChild = uint8_t;
  using Position = Eigen::Matrix<Element, dim, 1>;

  static constexpr Element kDim = dim;
  static constexpr RelativeChild kNumChildren = int_math::exp2(dim);

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
  template <Element max_height>
  std::vector<NdtreeIndex> computeParentIndices() const;

  NdtreeIndex computeChildIndex(RelativeChild relative_child_index) const;
  using ChildArray = std::array<NdtreeIndex, kNumChildren>;
  ChildArray computeChildIndices() const;

  RelativeChild computeRelativeChildIndex() const;
  template <typename NdtreeIndex<dim>::Element max_height>
  std::vector<RelativeChild> computeRelativeChildIndices() const;

  std::string toString() const;
};

using BinaryTreeIndex = NdtreeIndex<1>;
using QuadtreeIndex = NdtreeIndex<2>;
using OctreeIndex = NdtreeIndex<3>;
}  // namespace wavemap

#include "wavemap_common/indexing/impl/ndtree_index_inl.h"

#endif  // WAVEMAP_COMMON_INDEXING_NDTREE_INDEX_H_
