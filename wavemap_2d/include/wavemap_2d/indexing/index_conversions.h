#ifndef WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
#define WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/aabb.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/indexing/ndtree_index.h"
#include "wavemap_2d/utils/int_math.h"

namespace wavemap_2d::convert {
// TODO(victorr): Check styleguide on whether these classless methods names
//                should start with a capital
inline Index scaledPointToNearestIndex(const Point& point) {
  return (point - Vector::Constant(0.5f)).array().round().cast<IndexElement>();
}

inline Index scaledPointToFloorIndex(const Point& point) {
  return (point - Vector::Constant(0.5f)).array().floor().cast<IndexElement>();
}

inline Index scaledPointToCeilIndex(const Point& point) {
  return (point - Vector::Constant(0.5f)).array().ceil().cast<IndexElement>();
}

inline Index pointToNearestIndex(const Point& point,
                                 FloatingPoint cell_width_inv) {
  return scaledPointToNearestIndex(point * cell_width_inv);
}

inline Index pointToFloorIndex(const Point& point,
                               FloatingPoint cell_width_inv) {
  return scaledPointToFloorIndex(point * cell_width_inv);
}

inline Index pointToCeilIndex(const Point& point,
                              FloatingPoint cell_width_inv) {
  return scaledPointToCeilIndex(point * cell_width_inv);
}

inline Point indexToMinCorner(const Index& index, FloatingPoint cell_width) {
  return index.cast<FloatingPoint>() * cell_width;
}

inline Point indexToCenterPoint(const Index& index, FloatingPoint cell_width) {
  return (index.cast<FloatingPoint>() + Vector::Constant(0.5f)) * cell_width;
}

inline Index indexToNewResolution(const Index& src_index,
                                  FloatingPoint src_cell_width,
                                  FloatingPoint dst_cell_width) {
  const Point center_point = indexToCenterPoint(src_index, src_cell_width);
  return pointToNearestIndex(center_point, 1.f / dst_cell_width);
}

inline FloatingPoint heightToCellWidth(FloatingPoint min_cell_width,
                                       QuadtreeIndex::Element height) {
  return min_cell_width * static_cast<FloatingPoint>(int_math::exp2(height));
}

inline QuadtreeIndex pointToNodeIndex(const Point& point,
                                      FloatingPoint min_cell_width,
                                      QuadtreeIndex::Element height) {
  const FloatingPoint node_width = heightToCellWidth(min_cell_width, height);
  const Index position_index = pointToNearestIndex(point, 1.f / node_width);
  return {height, position_index};
}

inline Point nodeIndexToCenterPoint(const QuadtreeIndex& node_index,
                                    FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return indexToCenterPoint(node_index.position, node_width);
}

inline Point nodeIndexToMinCorner(const QuadtreeIndex& node_index,
                                  FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return indexToMinCorner(node_index.position, node_width);
}

inline Point nodeIndexToMaxCorner(const QuadtreeIndex& node_index,
                                  FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return node_index.position.cast<FloatingPoint>() * node_width +
         Vector::Constant(node_width);
}

inline AABB<Point> nodeIndexToAABB(const QuadtreeIndex& node_index,
                                   FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  const Point min_corner = indexToMinCorner(node_index.position, node_width);
  const Point max_corner = min_corner + Vector::Constant(node_width);
  return {min_corner, max_corner};
}

inline QuadtreeIndex indexAndHeightToNodeIndex(const Index& index,
                                               QuadtreeIndex::Element height) {
  DCHECK_GE(height, 0);
  QuadtreeIndex node_index{height, index};
  node_index.position = int_math::div_exp2(node_index.position, height);
  return node_index;
}

inline Index nodeIndexToIndex(const QuadtreeIndex& node_index) {
  DCHECK_GE(node_index.height, 0);
  Index index = int_math::mult_exp2(node_index.position, node_index.height);
  return index;
}
}  // namespace wavemap_2d::convert

#endif  // WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
