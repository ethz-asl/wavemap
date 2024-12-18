#ifndef WAVEMAP_CORE_INDEXING_INDEX_CONVERSIONS_H_
#define WAVEMAP_CORE_INDEXING_INDEX_CONVERSIONS_H_

#include <algorithm>
#include <limits>
#include <numeric>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/utils/bits/morton_encoding.h"
#include "wavemap/core/utils/data/eigen_checks.h"
#include "wavemap/core/utils/math/int_math.h"
#include "wavemap/core/utils/shape/aabb.h"

namespace wavemap::convert {
template <int dim>
inline Index<dim> scaledPointToNearestIndex(const Point<dim>& point) {
  return (point.array() - 0.5f).round().template cast<IndexElement>();
}

template <int dim>
inline Index<dim> scaledPointToFloorIndex(const Point<dim>& point) {
  return (point.array() - 0.5f).floor().template cast<IndexElement>();
}

template <int dim>
inline Index<dim> scaledPointToCeilIndex(const Point<dim>& point) {
  return (point.array() - 0.5f).ceil().template cast<IndexElement>();
}

template <int dim>
inline Index<dim> pointToNearestIndex(const Point<dim>& point,
                                      FloatingPoint cell_width_inv) {
  return scaledPointToNearestIndex<dim>(point * cell_width_inv);
}

template <int dim>
inline Index<dim> pointToFloorIndex(const Point<dim>& point,
                                    FloatingPoint cell_width_inv) {
  return scaledPointToFloorIndex<dim>(point * cell_width_inv);
}

template <int dim>
inline Index<dim> pointToCeilIndex(const Point<dim>& point,
                                   FloatingPoint cell_width_inv) {
  return scaledPointToCeilIndex<dim>(point * cell_width_inv);
}

template <int dim>
inline Point<dim> indexToMinCorner(const Index<dim>& index,
                                   FloatingPoint cell_width) {
  return index.template cast<FloatingPoint>() * cell_width;
}

template <int dim>
inline Point<dim> indexToMaxCorner(const Index<dim>& index,
                                   FloatingPoint cell_width) {
  return (index.template cast<FloatingPoint>().array() + 1.f) * cell_width;
}

template <int dim>
inline Point<dim> indexToCenterPoint(const Index<dim>& index,
                                     FloatingPoint cell_width) {
  return (index.template cast<FloatingPoint>().array() + 0.5f) * cell_width;
}

template <int dim>
inline Index<dim> indexToNewResolution(const Index<dim>& src_index,
                                       FloatingPoint src_cell_width,
                                       FloatingPoint dst_cell_width) {
  const Point<dim> center_point = indexToCenterPoint(src_index, src_cell_width);
  return pointToNearestIndex(center_point, 1.f / dst_cell_width);
}

inline FloatingPoint heightToCellWidth(FloatingPoint min_cell_width,
                                       IndexElement height) {
  return min_cell_width * static_cast<FloatingPoint>(int_math::exp2(height));
}

inline IndexElement cellWidthToHeight(FloatingPoint cell_width,
                                      FloatingPoint min_cell_width_inv) {
  return std::ceil(std::log2(cell_width * min_cell_width_inv));
}

template <int cells_per_side, int dim>
inline LinearIndex indexToLinearIndex(const Index<dim>& index) {
  DCHECK_EIGEN_GE(index, Index<dim>::Zero());
  DCHECK_EIGEN_LT(index, Index<dim>::Constant(cells_per_side));
  constexpr auto pow_sequence =
      int_math::pow_sequence<IndexElement, cells_per_side, dim>();
  return std::transform_reduce(pow_sequence.cbegin(), pow_sequence.cend(),
                               index.data(), 0);
}

template <int cells_per_side, int dim>
inline Index<dim> linearIndexToIndex(LinearIndex linear_index) {
  DCHECK_GE(linear_index, 0);
  DCHECK_LT(linear_index, std::pow(cells_per_side, dim));
  constexpr auto pow_sequence =
      int_math::pow_sequence<IndexElement, cells_per_side, dim>();
  Index<dim> index;
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    index[dim_idx] = (linear_index / pow_sequence[dim_idx]) % cells_per_side;
  }
  return index;
}

template <int dim>
inline NdtreeIndex<dim> pointToNodeIndex(const Point<dim>& point,
                                         FloatingPoint min_cell_width,
                                         IndexElement height) {
  const FloatingPoint node_width = heightToCellWidth(min_cell_width, height);
  const Index<dim> position_index =
      pointToNearestIndex(point, 1.f / node_width);
  return {height, position_index};
}

template <int dim>
inline Point<dim> nodeIndexToCenterPoint(const NdtreeIndex<dim>& node_index,
                                         FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return indexToCenterPoint(node_index.position, node_width);
}

template <int dim>
inline Point<dim> nodeIndexToMinCorner(const NdtreeIndex<dim>& node_index,
                                       FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return indexToMinCorner(node_index.position, node_width);
}

template <int dim>
inline Point<dim> nodeIndexToMaxCorner(const NdtreeIndex<dim>& node_index,
                                       FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return indexToMaxCorner(node_index.position, node_width);
}

template <int dim>
inline AABB<Point<dim>> nodeIndexToAABB(const NdtreeIndex<dim>& node_index,
                                        FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  const Point<dim> min_corner =
      indexToMinCorner(node_index.position, node_width);
  const Point<dim> max_corner =
      indexToMaxCorner(node_index.position, node_width);
  return {min_corner, max_corner};
}

template <int dim>
inline NdtreeIndex<dim> indexAndHeightToNodeIndex(const Index<dim>& index,
                                                  IndexElement height) {
  DCHECK_GE(height, 0);
  NdtreeIndex<dim> node_index{height, index};
  node_index.position = int_math::div_exp2_floor(node_index.position, height);
  return node_index;
}

template <int dim>
inline Index<dim> nodeIndexToMinCornerIndex(
    const NdtreeIndex<dim>& node_index) {
  DCHECK_GE(node_index.height, 0);
  Index<dim> index =
      int_math::mult_exp2(node_index.position, node_index.height);
  return index;
}

template <int dim>
inline Index<dim> nodeIndexToMaxCornerIndex(
    const NdtreeIndex<dim>& node_index) {
  DCHECK_GE(node_index.height, 0);
  const IndexElement max_child_offset = int_math::exp2(node_index.height) - 1;
  Index<dim> index =
      nodeIndexToMinCornerIndex(node_index).array() + max_child_offset;
  return index;
}

template <int dim>
MortonIndex indexToMorton(const Index<dim>& index) {
  return morton::encode<dim>(index);
}

template <int dim>
Index<dim> mortonToIndex(MortonIndex morton) {
  return morton::decode<dim>(morton);
}

template <int dim>
inline MortonIndex nodeIndexToMorton(const NdtreeIndex<dim>& node_index) {
  return convert::indexToMorton(node_index.position)
         << (node_index.height * dim);
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_CORE_INDEXING_INDEX_CONVERSIONS_H_
