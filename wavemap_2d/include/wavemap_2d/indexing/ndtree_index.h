#ifndef WAVEMAP_2D_INDEXING_NDTREE_INDEX_H_
#define WAVEMAP_2D_INDEXING_NDTREE_INDEX_H_

#include <string>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/utils/constexpr_functions.h"

namespace wavemap_2d {
template <int dim>
struct NdtreeIndex {
  using Element = int;
  using RelativeChild = uint8_t;
  using Position = Eigen::Matrix<Element, dim, 1>;

  static constexpr Element kDim = dim;
  static constexpr RelativeChild kNumChildren = constexpr_functions::exp2(dim);

  Element depth = 0;
  Position position = Position::Zero();

  bool operator==(const NdtreeIndex& other) const {
    return depth == other.depth && position == other.position;
  }
  bool operator!=(const NdtreeIndex& other) const {
    return !(*this == other);  // NOLINT
  }

  NdtreeIndex computeParentIndex() const;
  NdtreeIndex computeParentIndex(Element parent_depth) const;
  std::vector<NdtreeIndex> computeParentIndices() const;

  NdtreeIndex computeChildIndex(RelativeChild relative_child_index) const;
  std::vector<NdtreeIndex> computeChildIndices() const;
  RelativeChild computeRelativeChildIndex() const;
  std::vector<RelativeChild> computeRelativeChildIndices() const;

  std::string toString() const;
};

using BinaryTreeIndex = NdtreeIndex<1>;
using QuadtreeIndex = NdtreeIndex<2>;
using OctreeIndex = NdtreeIndex<3>;
}  // namespace wavemap_2d

#include "wavemap_2d/indexing/impl/ndtree_index_inl.h"

#endif  // WAVEMAP_2D_INDEXING_NDTREE_INDEX_H_
