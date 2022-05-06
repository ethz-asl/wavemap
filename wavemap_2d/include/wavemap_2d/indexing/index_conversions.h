#ifndef WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_
#define WAVEMAP_2D_INDEXING_INDEX_CONVERSIONS_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
// TODO(victorr): Check styleguide on whether these classless methods names
//                should start with a capital
inline Index computeNearestIndexForPoint(const Point& point,
                                         FloatingPoint resolution_inv) {
  return (point * resolution_inv).array().round().cast<IndexElement>();
}

inline Index computeNearestIndexForScaledPoint(const Point& point) {
  return point.array().round().cast<IndexElement>();
}

inline Index computeFloorIndexForPoint(const Point& point,
                                       FloatingPoint resolution_inv) {
  return (point * resolution_inv).array().floor().cast<IndexElement>();
}

inline Index computeCeilIndexForPoint(const Point& point,
                                      FloatingPoint resolution_inv) {
  return (point * resolution_inv).array().ceil().cast<IndexElement>();
}

inline Point computeCenterFromIndex(const Index& index,
                                    FloatingPoint resolution) {
  return index.cast<FloatingPoint>() * resolution;
}

inline QuadtreeIndex computeNodeIndexFromCenter(const Point& center,
                                                FloatingPoint root_node_width,
                                                QuadtreeIndex::Element depth) {
  const FloatingPoint width =
      root_node_width / std::exp2f(static_cast<FloatingPoint>(depth));
  // TODO(victorr): This offset is data structure specific (move it to the ds)
  const Vector root_node_halved_diagonal =
      root_node_width * Vector::Constant(0.5f);
  Index position_index = computeNearestIndexForScaledPoint(
      (center + root_node_halved_diagonal) / width);
  return {depth, position_index};
}

inline Point computeNodeCenterFromNodeIndex(const QuadtreeIndex& node_index,
                                            FloatingPoint root_node_width) {
  const FloatingPoint width =
      root_node_width /
      std::exp2f(static_cast<FloatingPoint>(node_index.depth));
  // TODO(victorr): This offset is data structure specific (move it to the ds)
  const Vector root_node_halved_diagonal =
      root_node_width * Vector::Constant(0.5f);
  return node_index.position.cast<FloatingPoint>() * width -
         root_node_halved_diagonal;
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
