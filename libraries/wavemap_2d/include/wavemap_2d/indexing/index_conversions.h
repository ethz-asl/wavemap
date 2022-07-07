#ifndef WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
#define WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/aabb.h>
#include <wavemap_common/indexing/ndtree_index.h>
#include <wavemap_common/utils/int_math.h>

namespace wavemap::convert {
// TODO(victorr): Check styleguide on whether these classless methods names
//                should start with a capital
inline Index2D scaledPointToNearestIndex(const Point2D& point) {
  return (point - Vector2D::Constant(0.5f))
      .array()
      .round()
      .cast<IndexElement>();
}

inline Index2D scaledPointToFloorIndex(const Point2D& point) {
  return (point - Vector2D::Constant(0.5f))
      .array()
      .floor()
      .cast<IndexElement>();
}

inline Index2D scaledPointToCeilIndex(const Point2D& point) {
  return (point - Vector2D::Constant(0.5f)).array().ceil().cast<IndexElement>();
}

inline Index2D pointToNearestIndex(const Point2D& point,
                                   FloatingPoint cell_width_inv) {
  return scaledPointToNearestIndex(point * cell_width_inv);
}

inline Index2D pointToFloorIndex(const Point2D& point,
                                 FloatingPoint cell_width_inv) {
  return scaledPointToFloorIndex(point * cell_width_inv);
}

inline Index2D pointToCeilIndex(const Point2D& point,
                                FloatingPoint cell_width_inv) {
  return scaledPointToCeilIndex(point * cell_width_inv);
}

inline Point2D indexToMinCorner(const Index2D& index,
                                FloatingPoint cell_width) {
  return index.cast<FloatingPoint>() * cell_width;
}

inline Point2D indexToCenterPoint(const Index2D& index,
                                  FloatingPoint cell_width) {
  return (index.cast<FloatingPoint>() + Vector2D::Constant(0.5f)) * cell_width;
}

inline Index2D indexToNewResolution(const Index2D& src_index,
                                    FloatingPoint src_cell_width,
                                    FloatingPoint dst_cell_width) {
  const Point2D center_point = indexToCenterPoint(src_index, src_cell_width);
  return pointToNearestIndex(center_point, 1.f / dst_cell_width);
}

inline FloatingPoint heightToCellWidth(FloatingPoint min_cell_width,
                                       QuadtreeIndex::Element height) {
  return min_cell_width * static_cast<FloatingPoint>(int_math::exp2(height));
}

inline QuadtreeIndex pointToNodeIndex(const Point2D& point,
                                      FloatingPoint min_cell_width,
                                      QuadtreeIndex::Element height) {
  const FloatingPoint node_width = heightToCellWidth(min_cell_width, height);
  const Index2D position_index = pointToNearestIndex(point, 1.f / node_width);
  return {height, position_index};
}

inline Point2D nodeIndexToCenterPoint(const QuadtreeIndex& node_index,
                                      FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return indexToCenterPoint(node_index.position, node_width);
}

inline Point2D nodeIndexToMinCorner(const QuadtreeIndex& node_index,
                                    FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return indexToMinCorner(node_index.position, node_width);
}

inline Point2D nodeIndexToMaxCorner(const QuadtreeIndex& node_index,
                                    FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  return node_index.position.cast<FloatingPoint>() * node_width +
         Vector2D::Constant(node_width);
}

inline AABB<Point2D> nodeIndexToAABB(const QuadtreeIndex& node_index,
                                     FloatingPoint min_cell_width) {
  const FloatingPoint node_width =
      heightToCellWidth(min_cell_width, node_index.height);
  const Point2D min_corner = indexToMinCorner(node_index.position, node_width);
  const Point2D max_corner = min_corner + Vector2D::Constant(node_width);
  return {min_corner, max_corner};
}

inline QuadtreeIndex indexAndHeightToNodeIndex(const Index2D& index,
                                               QuadtreeIndex::Element height) {
  DCHECK_GE(height, 0);
  QuadtreeIndex node_index{height, index};
  node_index.position = int_math::div_exp2(node_index.position, height);
  return node_index;
}

inline Index2D nodeIndexToMinCornerIndex(const QuadtreeIndex& node_index) {
  DCHECK_GE(node_index.height, 0);
  Index2D index = int_math::mult_exp2(node_index.position, node_index.height);
  return index;
}

inline Index2D nodeIndexToMaxCornerIndex(const QuadtreeIndex& node_index) {
  DCHECK_GE(node_index.height, 0);
  const QuadtreeIndex::Element max_child_offset =
      int_math::exp2(node_index.height) - 1;
  Index2D index =
      nodeIndexToMinCornerIndex(node_index).array() + max_child_offset;
  return index;
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
