#ifndef WAVEMAP_DATA_STRUCTURE_DENSE_GRID_H_
#define WAVEMAP_DATA_STRUCTURE_DENSE_GRID_H_

#include "wavemap/common.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/utils/data/eigen_checks.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap {
template <typename CellDataT, int dim, unsigned cells_per_side>
class DenseGrid {
 private:
  static_assert(int_math::is_power_of_two(cells_per_side),
                "Cells per side must be an exact power of 2.");
  static constexpr IndexElement kCellsPerSideLog2 =
      int_math::log2_floor(cells_per_side);

 public:
  static constexpr IndexElement kDim = dim;
  static constexpr IndexElement kCellsPerSide = cells_per_side;
  static constexpr IndexElement kCellsPerBlock =
      int_math::exp2(dim * kCellsPerSideLog2);

  using DataArrayType = std::array<FloatingPoint, kCellsPerBlock>;

  explicit DenseGrid(const CellDataT& default_value) {
    if (default_value != CellDataT{}) {
      data_.fill(default_value);
    }
  }

  CellDataT& operator[](size_t linear_index) {
    DCHECK_GE(linear_index, 0);
    DCHECK_LT(linear_index, kCellsPerBlock);
    return data_[linear_index];
  }
  const CellDataT& operator[](size_t linear_index) const {
    DCHECK_GE(linear_index, 0);
    DCHECK_LT(linear_index, kCellsPerBlock);
    return data_[linear_index];
  }

  CellDataT& at(const Index<dim>& index) {
    DCHECK_EIGEN_GE(index, Index<dim>::Zero());
    DCHECK_EIGEN_LT(index, Index<dim>::Constant(kCellsPerSide));
    return data_[convert::indexToLinearIndex<kCellsPerSide, dim>(index)];
  }
  const CellDataT& at(const Index<dim>& index) const {
    DCHECK_EIGEN_GE(index, Index<dim>::Zero());
    DCHECK_EIGEN_LT(index, Index<dim>::Constant(kCellsPerSide));
    return data_[convert::indexToLinearIndex<kCellsPerSide, dim>(index)];
  }

  DataArrayType& data() { return data_; }
  const DataArrayType& data() const { return data_; }

 private:
  DataArrayType data_{};
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_DENSE_GRID_H_
