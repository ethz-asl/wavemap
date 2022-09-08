#ifndef WAVEMAP_COMMON_INDEXING_INDEX_CONVERSIONS_H_
#define WAVEMAP_COMMON_INDEXING_INDEX_CONVERSIONS_H_

#include <numeric>

#include "wavemap_common/common.h"
#include "wavemap_common/data_structure/aabb.h"
#include "wavemap_common/indexing/ndtree_index.h"
#include "wavemap_common/utils/int_math.h"

namespace wavemap::convert {
// TODO(victorr): Check styleguide on whether these classless methods names
//                should start with a capital
template <int dim>
inline Index<dim> scaledPointToNearestIndex(const Point<dim>& point) {
  return (point - Vector<dim>::Constant(0.5f))
      .array()
      .round()
      .template cast<IndexElement>();
}

template <int dim>
inline Index<dim> scaledPointToFloorIndex(const Point<dim>& point) {
  return (point - Vector<dim>::Constant(0.5f))
      .array()
      .floor()
      .template cast<IndexElement>();
}

template <int dim>
inline Index<dim> scaledPointToCeilIndex(const Point<dim>& point) {
  return (point - Vector<dim>::Constant(0.5f))
      .array()
      .ceil()
      .template cast<IndexElement>();
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
inline Point<dim> indexToCenterPoint(const Index<dim>& index,
                                     FloatingPoint cell_width) {
  return (index.template cast<FloatingPoint>() + Vector<dim>::Constant(0.5f)) *
         cell_width;
}

template <int dim>
inline Index<dim> indexToNewResolution(const Index<dim>& src_index,
                                       FloatingPoint src_cell_width,
                                       FloatingPoint dst_cell_width) {
  const Point<dim> center_point = indexToCenterPoint(src_index, src_cell_width);
  return pointToNearestIndex(center_point, 1.f / dst_cell_width);
}

inline FloatingPoint heightToCellWidth(FloatingPoint min_cell_width,
                                       NdtreeIndexElement height) {
  return min_cell_width * static_cast<FloatingPoint>(int_math::exp2(height));
}

template <int cells_per_side, int dim>
inline LinearIndex indexToLinearIndex(const Index<dim>& index) {
  DCHECK((0 <= index.array() && index.array() < cells_per_side).all());
  constexpr auto pow_sequence =
      int_math::pow_sequence<IndexElement, cells_per_side, dim>();
  return std::transform_reduce(pow_sequence.begin(), pow_sequence.end(),
                               index.data(), 0);
}

template <int cells_per_side, int dim>
inline Index<dim> linearIndexToIndex(LinearIndex linear_index) {
  DCHECK(linear_index < std::pow(cells_per_side, dim));
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
                                         NdtreeIndexElement height) {
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
  return node_index.position.template cast<FloatingPoint>() * node_width +
         Vector<dim>::Constant(node_width);
}

template <int dim>
inline AABB<Point<dim>> nodeIndexToAABB(const NdtreeIndex<dim>& node_index,
                                        FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  const Point<dim> min_corner =
      indexToMinCorner(node_index.position, node_width);
  const Point<dim> max_corner = min_corner + Vector<dim>::Constant(node_width);
  return {min_corner, max_corner};
}

template <int dim>
inline NdtreeIndex<dim> indexAndHeightToNodeIndex(const Index<dim>& index,
                                                  NdtreeIndexElement height) {
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
  const NdtreeIndexElement max_child_offset =
      int_math::exp2(node_index.height) - 1;
  Index<dim> index =
      nodeIndexToMinCornerIndex(node_index).array() + max_child_offset;
  return index;
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_COMMON_INDEXING_INDEX_CONVERSIONS_H_
