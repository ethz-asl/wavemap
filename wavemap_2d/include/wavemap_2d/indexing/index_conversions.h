#ifndef WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
#define WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/aabb.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/indexing/ndtree_index.h"

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
                                 FloatingPoint resolution_inv) {
  return scaledPointToNearestIndex(point * resolution_inv);
}

inline Index pointToFloorIndex(const Point& point,
                               FloatingPoint resolution_inv) {
  return scaledPointToFloorIndex(point * resolution_inv);
}

inline Index pointToCeilIndex(const Point& point,
                              FloatingPoint resolution_inv) {
  return scaledPointToCeilIndex(point * resolution_inv);
}

inline Point indexToCenterPoint(const Index& index, FloatingPoint resolution) {
  return (index.cast<FloatingPoint>() + Vector::Constant(0.5f)) * resolution;
}

inline Index indexToNewResolution(const Index& src_index,
                                  FloatingPoint src_resolution,
                                  FloatingPoint dst_resolution) {
  const Point center_point = indexToCenterPoint(src_index, src_resolution);
  return pointToNearestIndex(center_point, 1.f / dst_resolution);
}

inline QuadtreeIndex pointToNodeIndex(const Point& point,
                                      FloatingPoint root_node_width,
                                      QuadtreeIndex::Element depth) {
  const auto exp2_depth = static_cast<FloatingPoint>(1 << depth);
  const FloatingPoint node_width_inv = exp2_depth / root_node_width;
  const FloatingPoint half_root_width_scaled = 0.5f * exp2_depth;
  constexpr FloatingPoint half_node_width_scaled = 0.5f;
  const Point scaled_point =
      point * node_width_inv +
      Vector::Constant(half_root_width_scaled - half_node_width_scaled);
  Index position_index = scaled_point.array().round().cast<IndexElement>();
  return {depth, position_index};
}

// TODO(victorr): Parameterize on height and max_resolution instead
inline Point nodeIndexToCenterPoint(const QuadtreeIndex& node_index,
                                    FloatingPoint root_node_width) {
  const FloatingPoint half_root_width = 0.5f * root_node_width;
  const FloatingPoint node_width =
      root_node_width / static_cast<FloatingPoint>(1 << node_index.depth);
  const FloatingPoint half_node_width = 0.5f * node_width;
  return node_index.position.cast<FloatingPoint>() * node_width +
         Vector::Constant(half_node_width - half_root_width);
}

inline Point nodeIndexToMinCorner(const QuadtreeIndex& node_index,
                                  FloatingPoint root_node_width) {
  const FloatingPoint half_root_width = 0.5f * root_node_width;
  const FloatingPoint node_width =
      root_node_width / static_cast<FloatingPoint>(1 << node_index.depth);
  return node_index.position.cast<FloatingPoint>() * node_width -
         Vector::Constant(half_root_width);
}

inline Point nodeIndexToMaxCorner(const QuadtreeIndex& node_index,
                                  FloatingPoint root_node_width) {
  const FloatingPoint half_root_width = 0.5f * root_node_width;
  const FloatingPoint node_width =
      root_node_width / static_cast<FloatingPoint>(1 << node_index.depth);
  return node_index.position.cast<FloatingPoint>() * node_width +
         Vector::Constant(node_width - half_root_width);
}

inline AABB<Point> nodeIndexToAABB(const QuadtreeIndex& node_index,
                                   FloatingPoint root_node_width) {
  const FloatingPoint half_root_width = 0.5f * root_node_width;
  const FloatingPoint node_width =
      root_node_width / static_cast<FloatingPoint>(1 << node_index.depth);
  const Point min_corner =
      node_index.position.cast<FloatingPoint>() * node_width -
      Vector::Constant(half_root_width);
  const Point max_corner = min_corner + Vector::Constant(node_width);
  return {min_corner, max_corner};
}

// TODO(victorr): Consider parameterizing nodes on height ipv depth
inline QuadtreeIndex indexAndDepthToNodeIndex(
    const Index& index, QuadtreeIndex::Element depth,
    QuadtreeIndex::Element max_depth) {
  DCHECK_LE(depth, max_depth);
  const QuadtreeIndex::Element node_height = max_depth - depth;
  QuadtreeIndex node_index{depth, index};
  node_index.position.x() >>= node_height;
  node_index.position.y() >>= node_height;
  return node_index;
}

inline Index nodeIndexToIndex(const QuadtreeIndex& node_index,
                              QuadtreeIndex::Element max_depth) {
  DCHECK_LE(node_index.depth, max_depth);
  const QuadtreeIndex::Element node_height = max_depth - node_index.depth;
  Index index = node_index.position * (1 << node_height);
  return index;
}
}  // namespace wavemap_2d::convert

#endif  // WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
