#ifndef WAVEMAP_UTILS_SDF_VECTOR_DISTANCE_FIELD_H_
#define WAVEMAP_UTILS_SDF_VECTOR_DISTANCE_FIELD_H_

#include <limits>

#include "wavemap/common.h"
#include "wavemap/data_structure/dense_grid.h"
#include "wavemap/data_structure/spatial_hash.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap {
struct VectorDistance {
  Index3D parent = Index3D::Constant(std::numeric_limits<IndexElement>::max());
  FloatingPoint distance = 0.f;

  friend bool operator==(const VectorDistance& lhs, const VectorDistance& rhs) {
    return lhs.parent == rhs.parent && lhs.distance == rhs.distance;
  }
  friend bool operator!=(const VectorDistance& lhs, const VectorDistance& rhs) {
    return !(lhs == rhs);
  }
};

class VectorDistanceField {
 private:
  static constexpr IndexElement kCellsPerSideLog2 = 4;
  static constexpr IndexElement kCellsPerSide =
      int_math::exp2(kCellsPerSideLog2);
  static constexpr IndexElement kDim = 3;

 public:
  using BlockIndex = Index3D;
  using CellIndex = Index3D;
  using Block = DenseGrid<VectorDistance, kDim, kCellsPerSide>;
  using BlockHashMap = SpatialHash<Block, kDim>;

  explicit VectorDistanceField(FloatingPoint min_cell_width,
                               FloatingPoint default_value = 0.f)
      : min_cell_width_(min_cell_width),
        default_value_({VectorDistance{}.parent, default_value}) {}

  const VectorDistance& getCellValue(const Index3D& index) const;
  VectorDistance& getCellValueRef(const Index3D& index);
  const VectorDistance& getDefaultCellValue() const { return default_value_; }

  Block* getBlock(const Index3D& block_index);
  const Block* getBlock(const Index3D& block_index) const;
  Block& getOrAllocateBlock(const Index3D& block_index);

  template <typename IndexedLeafVisitorFunction>
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const;

 private:
  const FloatingPoint min_cell_width_;
  const VectorDistance default_value_;
  BlockHashMap block_map_;

  static Index3D indexToBlockIndex(const Index3D& index);
  static Index3D indexToCellIndex(const Index3D& index);
  static Index3D cellAndBlockIndexToIndex(const Index3D& block_index,
                                          const Index3D& cell_index);
};
}  // namespace wavemap

#include "wavemap/utils/sdf/impl/vector_distance_field_inl.h"

#endif  // WAVEMAP_UTILS_SDF_VECTOR_DISTANCE_FIELD_H_
