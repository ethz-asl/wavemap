#ifndef WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
#define WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
// TODO(victorr): Check styleguide on whether these classless methods names
//                should start with a capital
inline Index computeNearestIndexFromScaledPoint(const Point& point) {
  return (point - Vector::Constant(0.5f)).array().round().cast<IndexElement>();
}

inline Index computeFloorIndexFromScaledPoint(const Point& point) {
  return (point - Vector::Constant(0.5f)).array().floor().cast<IndexElement>();
}

inline Index computeCeilIndexFromScaledPoint(const Point& point) {
  return (point - Vector::Constant(0.5f)).array().ceil().cast<IndexElement>();
}

inline Index computeNearestIndexFromPoint(const Point& point,
                                          FloatingPoint resolution_inv) {
  return computeNearestIndexFromScaledPoint(point * resolution_inv);
}

inline Index computeFloorIndexFromPoint(const Point& point,
                                        FloatingPoint resolution_inv) {
  return computeFloorIndexFromScaledPoint(point * resolution_inv);
}

inline Index computeCeilIndexFromPoint(const Point& point,
                                       FloatingPoint resolution_inv) {
  return computeCeilIndexFromScaledPoint(point * resolution_inv);
}

inline Point computeCenterFromIndex(const Index& index,
                                    FloatingPoint resolution) {
  return (index.cast<FloatingPoint>() + Vector::Constant(0.5f)) * resolution;
}

inline Index convertIndex(const Index& src_index, FloatingPoint src_resolution,
                          FloatingPoint dst_resolution) {
  const Point center_point = computeCenterFromIndex(src_index, src_resolution);
  return computeNearestIndexFromPoint(center_point, 1.f / dst_resolution);
}

inline QuadtreeIndex computeNodeIndexFromPoint(const Point& center,
                                               FloatingPoint root_node_width,
                                               QuadtreeIndex::Element depth) {
  const auto exp2_depth = static_cast<FloatingPoint>(1 << depth);
  const FloatingPoint node_width_inv = exp2_depth / root_node_width;
  const FloatingPoint half_root_width_scaled = 0.5f * exp2_depth;
  constexpr FloatingPoint half_node_width_scaled = 0.5f;
  const Point scaled_point =
      center * node_width_inv +
      Vector::Constant(half_root_width_scaled - half_node_width_scaled);
  Index position_index = scaled_point.array().round().cast<IndexElement>();
  return {depth, position_index};
}

inline Point computeNodeMinCornerFromNodeIndex(const QuadtreeIndex& node_index,
                                               FloatingPoint root_node_width) {
  const FloatingPoint half_root_width = 0.5f * root_node_width;
  const FloatingPoint node_width =
      root_node_width / static_cast<FloatingPoint>(1 << node_index.depth);
  return node_index.position.cast<FloatingPoint>() * node_width -
         Vector::Constant(half_root_width);
}

inline Point computeNodeCenterFromNodeIndex(const QuadtreeIndex& node_index,
                                            FloatingPoint root_node_width) {
  const FloatingPoint half_root_width = 0.5f * root_node_width;
  const FloatingPoint node_width =
      root_node_width / static_cast<FloatingPoint>(1 << node_index.depth);
  const FloatingPoint half_node_width = 0.5f * node_width;
  return node_index.position.cast<FloatingPoint>() * node_width +
         Vector::Constant(half_node_width - half_root_width);
}

// TODO(victorr): Consider parameterizing nodes on height ipv depth
inline QuadtreeIndex computeNodeIndexFromIndexAndDepth(
    const Index& index, QuadtreeIndex::Element depth,
    QuadtreeIndex::Element max_depth) {
  DCHECK_LE(depth, max_depth);
  const QuadtreeIndex::Element node_height = max_depth - depth;
  QuadtreeIndex node_index{depth, index};
  node_index.position.x() >>= node_height;
  node_index.position.y() >>= node_height;
  return node_index;
}

inline Index computeIndexFromNodeIndex(const QuadtreeIndex& node_index,
                                       QuadtreeIndex::Element max_depth) {
  DCHECK_LE(node_index.depth, max_depth);
  const QuadtreeIndex::Element node_height = max_depth - node_index.depth;
  Index index = node_index.position * (1 << node_height);
  return index;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
